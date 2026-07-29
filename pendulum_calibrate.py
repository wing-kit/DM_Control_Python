#!/usr/bin/env python3
"""
pendulum_calibrate.py — safe gravity calibration in POSITION mode.

Instead of sweeping under MIT control (which needs guessed gains), this uses the
motor's own position servo to visit known clock angles slowly, hold still, and
measure the holding torque. Static equilibrium => motor torque = gravity torque,
so a fit of tau = G*sin(q) + B over the sampled angles gives the gravity
feedforward for MIT-mode control — no guesswork, no dynamics.

Clock convention (zero at 12 o'clock, set with: dm_motor.py set-zero):
    0 deg   -> 12 o'clock (upright)
    +90 deg -> 3 o'clock
    180 deg -> 6 o'clock (hanging down)
    -90 deg -> 9 o'clock

Path: starts near 6 o'clock (stable), walks up the 3-o'clock side to 12,
down the 9-o'clock side, then returns the same way — every leg is <= 45 deg.
Repeated angles on the return leg reveal friction hysteresis.

Requirements:
  - Motor must be in POS_VEL mode (CTRL_MODE=2):
        .venv/bin/python dm_motor.py mode POS_VEL
        .venv/bin/python dm_motor.py save-params
  - Nothing within the arm's full swing radius.

Afterwards, use the printed G with the MIT controller:
    .venv/bin/python dm_motor.py mode MIT && .venv/bin/python dm_motor.py save-params
    .venv/bin/python pendulum_control.py --gravity <G> --gravity-offset <B>
"""

import argparse
import math
import sys
import time

import numpy as np

from DM_CAN import Motor, DM_Motor_Type, DM_variable
from DM_CAN_SocketCAN import MotorControlSocketCAN


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--channel", default="can0")
    p.add_argument("--type", default="DM10422P",
                   choices=[m.name for m in DM_Motor_Type])
    p.add_argument("--slave-id", type=lambda x: int(x, 0), default=1)
    p.add_argument("--master-id", type=lambda x: int(x, 0), default=0)
    p.add_argument("--speed", type=float, default=0.3,
                   help="slew speed between angles, rad/s (default 0.3 = slow)")
    p.add_argument("--settle", type=float, default=1.5,
                   help="settle time at each angle before sampling, s")
    p.add_argument("--sample-time", type=float, default=2.0,
                   help="torque sampling duration at each angle, s")
    p.add_argument("--move-timeout", type=float, default=25.0,
                   help="abort if a move takes longer than this, s")
    return p.parse_args()


# degrees; every consecutive leg <= 45 deg, starts and ends near 6 o'clock
WAYPOINTS_DEG = [135, 90, 45, 0, -45, -90, -45, 0, 45, 90, 135, 180]


def main():
    args = parse_args()
    control = MotorControlSocketCAN(args.channel)
    motor = Motor(DM_Motor_Type[args.type], SlaveID=args.slave_id, MasterID=args.master_id)
    control.addMotor(motor)

    # --- sanity: must be in POS_VEL mode ---
    mode = control.read_motor_param(motor, int(DM_variable.CTRL_MODE))
    if mode != 2:
        control.close()
        sys.exit(f"error: motor is in CTRL_MODE={mode}, not POS_VEL (2).\n"
                 f"  run: .venv/bin/python dm_motor.py mode POS_VEL && "
                 f".venv/bin/python dm_motor.py save-params")

    def poll():
        control.refresh_motor_status(motor)
        return float(motor.getPosition()), float(motor.getVelocity()), float(motor.getTorque())

    def abort(reason):
        print(f"\n[abort: {reason}] disabling motor")
        try:
            control.disable(motor)
        finally:
            control.close()
        sys.exit(1)

    print("enabling motor (position mode hold)...")
    control.enable(motor)

    q0, _, _ = poll()
    print(f"start position: {q0:.3f} rad ({math.degrees(q0):.0f} deg)")
    if abs(math.degrees(q0)) < 100:
        print("WARNING: arm is not near 6 o'clock; first move will be long. "
              "Consider disabling and letting it hang first.")

    samples = []  # (q_deg_target, q_meas, tau_mean, tau_std)
    try:
        for deg in WAYPOINTS_DEG:
            q_target = math.radians(deg)
            control.control_Pos_Vel(motor, P_desired=q_target, V_desired=args.speed)

            # wait for arrival
            t0 = time.monotonic()
            arrived = False
            while time.monotonic() - t0 < args.move_timeout:
                time.sleep(0.05)
                q, dq, _ = poll()
                if abs(dq) > 2.0:
                    abort(f"unexpected fast motion ({dq:+.2f} rad/s)")
                if abs(q - q_target) < 0.02 and abs(dq) < 0.05:
                    arrived = True
                    break
            if not arrived:
                abort(f"move to {deg} deg timed out (stuck at {math.degrees(q):.0f} deg)")

            # settle, then sample
            time.sleep(args.settle)
            taus, qs = [], []
            t0 = time.monotonic()
            while time.monotonic() - t0 < args.sample_time:
                q, _, tau = poll()
                qs.append(q)
                taus.append(tau)
                time.sleep(0.05)
            q_meas = float(np.mean(qs))
            tau_mean, tau_std = float(np.mean(taus)), float(np.std(taus))
            samples.append((deg, q_meas, tau_mean, tau_std))
            print(f"  {deg:+5d} deg ({deg//30 if deg>=0 else (deg-30)//30:>2} o'clock-ish): "
                  f"q={q_meas:+.4f} rad  tau={tau_mean:+.3f} ± {tau_std:.3f} N*m", flush=True)

    except KeyboardInterrupt:
        abort("Ctrl-C")

    # ---- fit tau = G*sin(q) + B ----
    q_arr = np.array([s[1] for s in samples])
    tau_arr = np.array([s[2] for s in samples])
    A = np.column_stack([np.sin(q_arr), np.ones(len(q_arr))])
    (G, B), *_ = np.linalg.lstsq(A, tau_arr, rcond=None)
    pred = A @ np.array([G, B])
    ss_res = float(np.sum((tau_arr - pred) ** 2))
    ss_tot = float(np.sum((tau_arr - np.mean(tau_arr)) ** 2)) or 1e-9
    r2 = 1.0 - ss_res / ss_tot

    # hysteresis from repeated angles (approached from opposite directions)
    by_deg = {}
    for deg, _, tau_mean, _ in samples:
        by_deg.setdefault(deg, []).append(tau_mean)
    hyst = [abs(v[0] - v[1]) for v in by_deg.values() if len(v) == 2]
    fc_est = float(np.mean(hyst)) / 2.0 if hyst else 0.0

    print("\n================ calibration result ================")
    print(f"  gravity constant  G = {G:+.3f} N*m  (per sin(rad) from upright)")
    print(f"  torque offset     B = {B:+.3f} N*m")
    print(f"  fit quality       R² = {r2:.4f}")
    print(f"  friction estimate fc ≈ {fc_est:.3f} N*m (half hysteresis, rough)")
    print("\n  check: 3 o'clock hold needs ~|G| N*m; if that looks wildly")
    print("  wrong vs the sampled tau at ±90 deg above, do NOT use it.")
    print("====================================================")
    print("\nnext steps:")
    print("  .venv/bin/python dm_motor.py mode MIT")
    print("  .venv/bin/python dm_motor.py save-params")
    print(f"  .venv/bin/python pendulum_control.py --gravity {G:.3f} --gravity-offset {B:.3f}")

    print("\nparking at 6 o'clock and disabling...")
    control.control_Pos_Vel(motor, P_desired=math.pi, V_desired=args.speed)
    time.sleep(3.0)
    control.disable(motor)
    control.close()
    print("done.")


if __name__ == "__main__":
    main()
