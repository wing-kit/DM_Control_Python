---
name: dm-motor
description: Control and monitor DaMiao (DM) BLDC motors over Linux SocketCAN using the dm_motor.py CLI in the DM_Control_Python repo. Use when reading motor position/velocity/torque feedback, scanning the CAN bus for motors, reading/writing motor registers, calibrating zero position, or commanding motion in position-velocity, velocity, or MIT modes.
---

# DaMiao Motor Control (SocketCAN)

## Environment

- Repo: `/home/longyu/DM_Control_Python` — run all commands from there.
- Always use the venv interpreter: `.venv/bin/python dm_motor.py ...`
- Hardware: one **DM10422P** motor, slave ID `1`, master ID `0`, on **can0** at 1 Mbps.
  These are the CLI defaults, so plain `dm_motor.py status` works.
- Motor limits: PMAX 12.566 rad, VMAX 20 rad/s, TMAX 500 N·m.
- If can0 is down (`OSError`): `sudo ip link set can0 up type can bitrate 1000000 restart-ms 100`
  (sudo needs no password on this machine).

## Safety rules — follow strictly

1. **Never send motion or write commands without the user's explicit request.**
   Reading (`scan`, `status`, `monitor`, `read-param`) is always safe.
2. Before any unfamiliar write/motion command, run it with `--dry-run` first to
   inspect the CAN frame, then confirm with the user before sending for real.
3. The motor only moves when **enabled**. `enable` makes it hold position — expect
   a small jolt. `disable` releases it (shaft goes limp).
4. Always keep moves gentle: velocity ≤ 5 rad/s unless the user asks otherwise.
5. `set-zero` and `save-params` write to the motor's flash and persist across
   power cycles. Only run them when the user explicitly confirms.
6. If feedback shows unexpected values or the motor faults, `disable` immediately.

## Command reference

All commands are one-shot and non-interactive. `--json` gives machine-readable
output on stdout (diagnostics go to stderr). Global flags work before or after
the subcommand: `--channel can0 --type DM10422P --slave-id 1 --master-id 0`.

### Read-only (always safe)

```bash
.venv/bin/python dm_motor.py scan                        # find motors on the bus
.venv/bin/python dm_motor.py status --json               # pos/vel/torque once
.venv/bin/python dm_motor.py monitor --rate 20 --duration 3   # stream feedback
.venv/bin/python dm_motor.py read-param PMAX VMAX TMAX   # registers by name...
.venv/bin/python dm_motor.py read-param 21 --json        # ...or by RID number
```

Useful register names: `UV_Value OT_Value OC_Value ACC DEC MAX_SPD MST_ID ESC_ID
TIMEOUT CTRL_MODE hw_ver sw_ver SN PMAX VMAX TMAX KP_ASR KI_ASR KP_APR KI_APR
can_br p_m xout` (full list: `DM_variable` enum in `DM_CAN.py`).

### State / configuration (write — confirm first)

```bash
.venv/bin/python dm_motor.py enable
.venv/bin/python dm_motor.py disable
.venv/bin/python dm_motor.py mode MIT            # MIT | POS_VEL | VEL | Torque_Pos
.venv/bin/python dm_motor.py write-param TIMEOUT 0
.venv/bin/python dm_motor.py save-params         # persist params to flash
.venv/bin/python dm_motor.py set-zero            # store CURRENT position as zero
```

### Motion (motor must be enabled; confirm with user first)

```bash
.venv/bin/python dm_motor.py pos-vel 0 1.0       # POS_VEL mode: go to 0 rad at 1 rad/s
.venv/bin/python dm_motor.py vel 0.5             # VEL mode: spin at 0.5 rad/s
.venv/bin/python dm_motor.py mit --kp 5 --kd 0.1 --q 0   # MIT mode: one command frame
.venv/bin/python dm_motor.py mit-stream --kp 10 --kd 0.5 --q 0   # MIT: hold position (streams at --rate Hz)
.venv/bin/python dm_motor.py mit-stream --amp 0.3 --freq 0.5 --duration 5  # MIT: sine sweep
```

MIT mode needs a continuous command stream — a single `mit` frame has no lasting
effect. Use `mit-stream` for anything real. MIT commands only work when
`CTRL_MODE=1` (switch with `mode MIT` + `save-params`).

Note: motion commands must match the motor's `CTRL_MODE` (check with
`read-param CTRL_MODE`: 1=MIT, 2=POS_VEL, 3=VEL, 4=Torque_Pos). Change with
`mode`, then `save-params` to persist.

## Common workflows

### Read current position

```bash
.venv/bin/python dm_motor.py status --json
# {"position_rad": 1.592058, "velocity_rad_s": -0.004884, "torque_Nm": -0.3663}
```

### Move to a target position (POS_VEL mode)

```bash
.venv/bin/python dm_motor.py enable
.venv/bin/python dm_motor.py pos-vel <target_rad> <speed_rad_s>
.venv/bin/python dm_motor.py monitor --rate 20 --duration 3   # watch it arrive
.venv/bin/python dm_motor.py disable                          # when done
```

### Zero calibration

1. `disable` so the shaft is free
2. User physically moves the shaft to the desired zero
3. `set-zero` (persists in motor flash)
4. `status` should read ~0 rad

### Change control mode

```bash
.venv/bin/python dm_motor.py mode MIT
.venv/bin/python dm_motor.py save-params
.venv/bin/python dm_motor.py read-param CTRL_MODE   # verify: 1
```

## Library API (for scripts beyond the CLI)

`DM_CAN_SocketCAN.py` provides `MotorControlSocketCAN(channel, dry_run=False)`
with the same API as the serial `MotorControl` in `DM_CAN.py`
(`enable/disable/controlMIT/control_Pos_Vel/control_Vel/control_pos_force/
read_motor_param/change_motor_param/switchControlMode/save_motor_param/
set_zero_position/refresh_motor_status`). Use it when a task needs tight
control loops (e.g. 100+ Hz MIT control) that one-shot CLI calls can't do.

**Protocol limits (critical):** the MIT frame encodes kp as 0-500 and kd as
0-5 in 12-bit fields. Values outside range corrupt the frame (fixed in
`float_to_uint`, but never rely on unclamped user input). Motors here have
large stiction (~5 N·m measured); weak kp (<10) may not move the arm at all.

See `README.md` section 7 for details and examples.

## Pendulum control (gravity-compensated MIT)

Two scripts implement a safe pendulum workflow (zero at 12 o'clock):

1. **Always calibrate first in POSITION mode** — never sweep-fit under MIT
   with guessed gains:
   ```bash
   .venv/bin/python dm_motor.py mode POS_VEL && .venv/bin/python dm_motor.py save-params
   .venv/bin/python pendulum_calibrate.py     # prints G, B, R², friction fc
   ```
2. **Then run MIT control with the measured values** (skips the risky sweep):
   ```bash
   .venv/bin/python dm_motor.py mode MIT && .venv/bin/python dm_motor.py save-params
   .venv/bin/python pendulum_control.py --gravity <G> --gravity-offset <B> \
       --kp 30 --kd 2.5 --ki 10 --i-max 12 --fc 3.0 --push-amp 0.3
   ```

Behavior notes for this joint: large coulomb friction (~5 N·m) causes a
two-phase push return (fast spring-back, then slow integral creep) — that is
normal; `--fc` friction feedforward and higher `--ki` speed it up. Over-
compensating `--fc` (>= real fc) causes buzzing around the target.
`pendulum_control.py` parks the arm at the bottom before disabling — do not
kill it with SIGKILL; use Ctrl-C and let it park.
