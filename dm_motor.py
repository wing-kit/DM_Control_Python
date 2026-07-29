#!/usr/bin/env python3
"""
dm_motor.py — CLI for DaMiao (DM) motor control over SocketCAN.

Designed for both humans and coding agents: every command is a one-shot,
non-interactive operation with optional machine-readable JSON output.

Examples (read-only, safe):
    ./dm_motor.py scan                          # find motors on the bus
    ./dm_motor.py status                        # pos/vel/tau once
    ./dm_motor.py status --json
    ./dm_motor.py monitor --rate 50 --duration 2
    ./dm_motor.py read-param PMAX
    ./dm_motor.py read-param 21 --json

Commands that write to the motor or cause motion (enable, disable, set-zero,
mode, write-param, save-params, mit, pos-vel, vel, pos-force) support
--dry-run, which prints the CAN frames instead of sending them:

    ./dm_motor.py --dry-run enable
    ./dm_motor.py --dry-run pos-vel 1.0 2.0

Global options (defaults match a single DM10422P, slave ID 1, on can0):
    --channel can0 --type DM10422P --slave-id 1 --master-id 0
"""

import argparse
import json
import select
import socket
import struct
import sys
import time

from DM_CAN import (
    Motor,
    DM_Motor_Type,
    DM_variable,
    Control_Type,
)
from DM_CAN_SocketCAN import MotorControlSocketCAN, CAN_FRAME_FMT

CONTROL_MODE_NAMES = {m.name: m for m in Control_Type}
MOTOR_TYPE_NAMES = {m.name: m for m in DM_Motor_Type}


# ----------------------------------------------------------------------
# helpers

def out(obj, as_json):
    if as_json:
        print(json.dumps(obj))
    else:
        if isinstance(obj, dict):
            for k, v in obj.items():
                print(f"{k}: {v}")
        else:
            print(obj)


def resolve_rid(rid_str):
    """accept a register name (PMAX) or integer (21)"""
    try:
        return int(rid_str, 0)
    except ValueError:
        pass
    key = rid_str.upper()
    for v in DM_variable:
        if v.name.upper() == key:
            return int(v)
    valid = ", ".join(v.name for v in DM_variable)
    raise SystemExit(f"unknown register '{rid_str}'. Valid names: {valid}")


def make_motor(args):
    mtype = MOTOR_TYPE_NAMES[args.type]
    return Motor(mtype, SlaveID=args.slave_id, MasterID=args.master_id)


def make_control(args):
    return MotorControlSocketCAN(args.channel, dry_run=args.dry_run)


def motor_state(motor):
    return {
        "position_rad": round(float(motor.getPosition()), 6),
        "velocity_rad_s": round(float(motor.getVelocity()), 6),
        "torque_Nm": round(float(motor.getTorque()), 6),
    }


# ----------------------------------------------------------------------
# commands

def cmd_scan(args):
    """probe slave IDs for responding motors (read-only)"""
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((args.channel,))
    s.setblocking(False)

    def send(can_id, data):
        s.send(struct.pack(CAN_FRAME_FMT, can_id, len(data), bytes(data).ljust(8, b"\x00")))

    found = []
    for mid in range(args.scan_min, args.scan_max + 1):
        send(0x7FF, [mid & 0xFF, (mid >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0])
        time.sleep(0.01)
        while True:
            r, _, _ = select.select([s], [], [], 0.02)
            if not r:
                break
            cid, dlc, data = struct.unpack(CAN_FRAME_FMT, s.recv(16))
            found.append({
                "slave_id": mid,
                "reply_can_id": cid & 0x7FF,
                "master_id": data[0] & 0x0F,
                "error_code": (data[0] >> 4) & 0x0F,
                "raw": data[:dlc].hex(),
            })
    s.close()
    if args.json:
        out(found, True)
    else:
        if not found:
            print(f"no motors found on {args.channel} (ids {args.scan_min}-{args.scan_max})")
        for f in found:
            print(f"motor: slave_id={f['slave_id']} master_id={f['master_id']} "
                  f"reply_can_id=0x{f['reply_can_id']:03X} err={f['error_code']} raw={f['raw']}")


def cmd_status(args):
    control = make_control(args)
    motor = make_motor(args)
    control.addMotor(motor)
    control.refresh_motor_status(motor)
    out(motor_state(motor), args.json)
    control.close()


def cmd_monitor(args):
    control = make_control(args)
    motor = make_motor(args)
    control.addMotor(motor)
    interval = 1.0 / args.rate
    t0 = time.monotonic()
    n = 0
    try:
        while True:
            control.refresh_motor_status(motor)
            st = motor_state(motor)
            st["t"] = round(time.monotonic() - t0, 4)
            out(st, args.json)
            n += 1
            if args.count and n >= args.count:
                break
            if args.duration and (time.monotonic() - t0) >= args.duration:
                break
            time.sleep(interval)
    except KeyboardInterrupt:
        pass
    control.close()


def cmd_read_param(args):
    control = make_control(args)
    motor = make_motor(args)
    control.addMotor(motor)
    result = {}
    for r in args.rids:
        rid = resolve_rid(r)
        val = control.read_motor_param(motor, rid)
        name = DM_variable(rid).name if rid in [int(v) for v in DM_variable] else str(rid)
        result[name] = None if val is None else float(val)
        if not args.json:
            print(f"{name} (RID {rid}): {val}")
    if args.json:
        out(result, True)
    control.close()


# ---- write / motion commands (respect --dry-run) ----

def cmd_enable(args):
    control = make_control(args)
    motor = make_motor(args)
    control.addMotor(motor)
    control.enable(motor)
    out({"enabled": True, **motor_state(motor)}, args.json)
    control.close()


def cmd_disable(args):
    control = make_control(args)
    motor = make_motor(args)
    control.addMotor(motor)
    control.disable(motor)
    out({"enabled": False, **motor_state(motor)}, args.json)
    control.close()


def cmd_set_zero(args):
    control = make_control(args)
    motor = make_motor(args)
    control.addMotor(motor)
    control.set_zero_position(motor)
    out({"zero_set": not args.dry_run, **motor_state(motor)}, args.json)
    control.close()


def cmd_mode(args):
    control = make_control(args)
    motor = make_motor(args)
    control.addMotor(motor)
    mode = CONTROL_MODE_NAMES[args.mode]
    if args.dry_run:
        # replicate the frame switchControlMode would send
        control._MotorControlSocketCAN__write_motor_param(motor, 10, int(mode))
        out({"dry_run": True, "mode": args.mode}, args.json)
    else:
        ok = control.switchControlMode(motor, mode)
        out({"mode": args.mode, "success": bool(ok)}, args.json)
    control.close()


def cmd_write_param(args):
    control = make_control(args)
    motor = make_motor(args)
    control.addMotor(motor)
    rid = resolve_rid(args.rid)
    if args.dry_run:
        control._MotorControlSocketCAN__write_motor_param(motor, rid, args.value)
        out({"dry_run": True, "rid": rid, "value": args.value}, args.json)
    else:
        ok = control.change_motor_param(motor, rid, args.value)
        out({"rid": rid, "value": args.value, "success": bool(ok)}, args.json)
    control.close()


def cmd_save_params(args):
    control = make_control(args)
    motor = make_motor(args)
    control.addMotor(motor)
    control.save_motor_param(motor)
    out({"saved": not args.dry_run}, args.json)
    control.close()


def cmd_mit(args):
    control = make_control(args)
    motor = make_motor(args)
    control.addMotor(motor)
    control.controlMIT(motor, kp=args.kp, kd=args.kd, q=args.q, dq=args.dq, tau=args.tau)
    out(motor_state(motor), args.json)
    control.close()


def cmd_pos_vel(args):
    control = make_control(args)
    motor = make_motor(args)
    control.addMotor(motor)
    control.control_Pos_Vel(motor, P_desired=args.position, V_desired=args.velocity)
    out(motor_state(motor), args.json)
    control.close()


def cmd_vel(args):
    control = make_control(args)
    motor = make_motor(args)
    control.addMotor(motor)
    control.control_Vel(motor, Vel_desired=args.velocity)
    out(motor_state(motor), args.json)
    control.close()


def cmd_pos_force(args):
    control = make_control(args)
    motor = make_motor(args)
    control.addMotor(motor)
    control.control_pos_force(motor, Pos_des=args.position, Vel_des=args.velocity_x100,
                              i_des=args.current_x10000)
    out(motor_state(motor), args.json)
    control.close()


# ----------------------------------------------------------------------
# argument parsing

def build_parser():
    p = argparse.ArgumentParser(
        prog="dm_motor",
        description="DaMiao motor CLI over SocketCAN (agent-friendly, one-shot commands)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    # common options: valid both before and after the subcommand.
    # main-parser copies carry the real defaults; subparser copies use
    # SUPPRESS so they only override when explicitly given.
    common = argparse.ArgumentParser(add_help=False)
    for parser, default in ((p, None), (common, argparse.SUPPRESS)):
        d = (lambda v: v) if default is None else (lambda v: default)
        parser.add_argument("--channel", default=d("can0"), help="SocketCAN interface (default: can0)")
        parser.add_argument("--type", default=d("DM10422P"), choices=sorted(MOTOR_TYPE_NAMES),
                            help="motor type (default: DM10422P)")
        parser.add_argument("--slave-id", type=lambda x: int(x, 0), default=d(1),
                            help="motor CAN slave ID (default: 1)")
        parser.add_argument("--master-id", type=lambda x: int(x, 0), default=d(0),
                            help="motor CAN master ID (default: 0)")
        parser.add_argument("--json", action="store_true", default=d(False),
                            help="machine-readable JSON output")
        parser.add_argument("--dry-run", action="store_true", default=d(False),
                            help="print CAN frames instead of sending them (safe testing)")

    sub = p.add_subparsers(dest="command", required=True)

    sp = sub.add_parser("scan", parents=[common], help="probe for motors on the bus (read-only)")
    sp.add_argument("--scan-min", type=int, default=1)
    sp.add_argument("--scan-max", type=int, default=32)
    sp.set_defaults(func=cmd_scan)

    sp = sub.add_parser("status", parents=[common], help="read position/velocity/torque once")
    sp.set_defaults(func=cmd_status)

    sp = sub.add_parser("monitor", parents=[common], help="stream position/velocity/torque")
    sp.add_argument("--rate", type=float, default=10.0, help="Hz (default 10)")
    sp.add_argument("--duration", type=float, default=0, help="seconds, 0=forever")
    sp.add_argument("--count", type=int, default=0, help="stop after N samples")
    sp.set_defaults(func=cmd_monitor)

    sp = sub.add_parser("read-param", parents=[common], help="read motor register(s) by name or RID")
    sp.add_argument("rids", nargs="+", help="e.g. PMAX VMAX TMAX or 21 22 23")
    sp.set_defaults(func=cmd_read_param)

    sp = sub.add_parser("enable", parents=[common], help="enable motor (CAUTION: motor may hold position)")
    sp.set_defaults(func=cmd_enable)

    sp = sub.add_parser("disable", parents=[common], help="disable motor")
    sp.set_defaults(func=cmd_disable)

    sp = sub.add_parser("set-zero", parents=[common], help="set current position as zero (writes to motor)")
    sp.set_defaults(func=cmd_set_zero)

    sp = sub.add_parser("mode", parents=[common], help="switch control mode (MIT, POS_VEL, VEL, Torque_Pos)")
    sp.add_argument("mode", choices=sorted(CONTROL_MODE_NAMES))
    sp.set_defaults(func=cmd_mode)

    sp = sub.add_parser("write-param", parents=[common], help="write motor register (use save-params to persist)")
    sp.add_argument("rid", help="register name or RID")
    sp.add_argument("value", type=float)
    sp.set_defaults(func=cmd_write_param)

    sp = sub.add_parser("save-params", parents=[common], help="save all parameters to flash (disables motor first)")
    sp.set_defaults(func=cmd_save_params)

    sp = sub.add_parser("mit", parents=[common], help="one MIT-mode command (MOTION!)")
    sp.add_argument("--kp", type=float, default=0.0)
    sp.add_argument("--kd", type=float, default=0.0)
    sp.add_argument("--q", type=float, default=0.0, help="target position rad")
    sp.add_argument("--dq", type=float, default=0.0, help="target velocity rad/s")
    sp.add_argument("--tau", type=float, default=0.0, help="feedforward torque N*m")
    sp.set_defaults(func=cmd_mit)

    sp = sub.add_parser("pos-vel", parents=[common], help="one position-velocity command (MOTION!)")
    sp.add_argument("position", type=float, help="rad")
    sp.add_argument("velocity", type=float, help="rad/s")
    sp.set_defaults(func=cmd_pos_vel)

    sp = sub.add_parser("vel", parents=[common], help="one velocity command (MOTION!)")
    sp.add_argument("velocity", type=float, help="rad/s")
    sp.set_defaults(func=cmd_vel)

    sp = sub.add_parser("pos-force", parents=[common], help="one force-position mixed command (MOTION!)")
    sp.add_argument("position", type=float, help="rad")
    sp.add_argument("velocity_x100", type=float, help="velocity * 100")
    sp.add_argument("current_x10000", type=float, help="normalized current * 10000, 0-10000")
    sp.set_defaults(func=cmd_pos_force)

    return p


def main():
    args = build_parser().parse_args()
    try:
        args.func(args)
    except OSError as e:
        print(f"error: {e} (is {args.channel} up? try: sudo ip link set {args.channel} up type can bitrate 1000000)",
              file=sys.stderr)
        sys.exit(2)


if __name__ == "__main__":
    main()
