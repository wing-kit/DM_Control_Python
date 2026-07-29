#!/usr/bin/env python3
"""
pendulum_control.py — auto-tuned, gravity-compensated pendulum control (MIT mode).

Setup assumption: the motor drives a pendulum arm, and its zero position is at
12 o'clock (arm pointing straight UP). Therefore:
    q = 0      -> 12 o'clock (upright, unstable)
    q = ±pi/2  -> 3 o'clock / 9 o'clock (horizontal)
    q = ±pi    -> 6 o'clock (hanging down, stable rest position)

Auto-tune stages (all slow and safe, small motions around the start position):
  1. GRAVITY + FRICTION — gentle sine sweep (low gains, amplitude ease-in,
     30 s period). Fits the full quasi-static/dynamic model by regression:
         tau(q, dq) = G*sin(q) + B + c*dq + fc*sign(dq)
     G/B are used as gravity feedforward; c/fc are reported for insight.
  2. RETURN TUNING — at the target angle, steps the reference by a small
     amount (like a controlled push) and watches the bounce-back. Adjusts kd
     automatically until the return is fast with < 5% overshoot.
  3. MOVE + HOLD — cosine-ramp to the target (default: 3 o'clock = +pi/2)
     with gravity feedforward + the tuned kp/kd/ki, then holds.
  4. PARK — on Ctrl-C or abort, smoothly swings back down to 6 o'clock
     (stable) before disabling, so the arm never drops.

Requirements:
  - Motor must already be in MIT mode (CTRL_MODE=1):
        .venv/bin/python dm_motor.py mode MIT && .venv/bin/python dm_motor.py save-params
  - Zero must be set at 12 o'clock (see: dm_motor.py set-zero).

If 3 o'clock is NEGATIVE rotation for your setup, use --target -1.5708.
Skip individual stages with --gravity G / --no-return-tune.
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

    p.add_argument("--target", type=float, default=math.pi / 2,
                   help="target angle rad from 12 o'clock (default: +pi/2 = 3 o'clock)")
    p.add_argument("--kp", type=float, default=20.0, help="MIT stiffness for hold (default 20)")
    p.add_argument("--kd", type=float, default=2.0, help="initial MIT damping; auto-refined (default 2)")
    p.add_argument("--ki", type=float, default=3.0,
                   help="integral gain on position error, N*m per rad*s (default 3)")
    p.add_argument("--i-max", type=float, default=8.0,
                   help="anti-windup clamp for the integral term, N*m")
    p.add_argument("--fc", type=float, default=0.0,
                   help="friction feedforward N*m: adds fc*tanh(err/eps) to break stiction. "
                        "Use LESS than the real stiction (e.g. 3.0 when fc~5) to avoid oscillation")
    p.add_argument("--fc-eps", type=float, default=0.02,
                   help="smoothing width for friction feedforward, rad")
    p.add_argument("--rate", type=float, default=200.0, help="control rate Hz")

    # stage 1: gravity/friction sweep (slow, but strong enough to actually move)
    p.add_argument("--tune-kp", type=float, default=40.0,
                   help="sweep stiffness; must be high enough to break stiction and track the sweep")
    p.add_argument("--tune-kd", type=float, default=3.0, help="sweep damping")
    p.add_argument("--tune-amp", type=float, default=0.5,
                   help="sweep amplitude rad around current position")
    p.add_argument("--tune-cycles", type=float, default=1.5)
    p.add_argument("--tune-period", type=float, default=30.0,
                   help="seconds per sweep cycle (slower = safer, more quasi-static)")
    p.add_argument("--gravity", type=float, default=None,
                   help="skip stage 1, use this G (N*m per sin(rad) from upright)")
    p.add_argument("--gravity-offset", type=float, default=0.0,
                   help="torque offset B (N*m) to use with --gravity, from pendulum_calibrate.py")

    # stage 2: push-recovery tuning
    p.add_argument("--no-return-tune", action="store_true",
                   help="skip stage 2, keep --kd as given")
    p.add_argument("--push-amp", type=float, default=0.12,
                   help="reference step size for push-recovery tests, rad")
    p.add_argument("--kd-min", type=float, default=0.3)
    p.add_argument("--kd-max", type=float, default=4.0,
                   help="MUST stay <= 5.0 (DaMiao MIT protocol kd limit)")
    p.add_argument("--tau-max", type=float, default=30.0,
                   help="hard clamp on total commanded torque, N*m (safety)")

    p.add_argument("--ramp-time", type=float, default=6.0,
                   help="seconds for the cosine ramp to target / to park")
    p.add_argument("--max-track-err", type=float, default=1.5,
                   help="abort if |q - q_ref| exceeds this (rad)")
    p.add_argument("--max-vel", type=float, default=4.0,
                   help="abort if |measured velocity| exceeds this (rad/s)")
    p.add_argument("--no-park", action="store_true",
                   help="on exit disable immediately instead of parking at the bottom (arm WILL drop)")
    return p.parse_args()


class Loop:
    """fixed-rate control loop helper"""
    def __init__(self, rate):
        self.dt = 1.0 / rate
        self.next = time.monotonic()
        self.t0 = self.next

    def tick(self):
        self.next += self.dt
        delay = self.next - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            self.next = time.monotonic()  # fell behind; resync
        return time.monotonic() - self.t0


def cosine_ramp(q_from, q_to, t, T):
    """position/velocity reference for a cosine (smooth start/stop) ramp"""
    if t >= T:
        return q_to, 0.0
    s = (1.0 - math.cos(math.pi * t / T)) / 2.0
    q = q_from + (q_to - q_from) * s
    dq = (q_to - q_from) * math.pi / (2.0 * T) * math.sin(math.pi * t / T)
    return q, dq


def main():
    args = parse_args()
    control = MotorControlSocketCAN(args.channel)
    motor = Motor(DM_Motor_Type[args.type], SlaveID=args.slave_id, MasterID=args.master_id)
    control.addMotor(motor)

    # --- sanity: must be in MIT mode ---
    mode = control.read_motor_param(motor, int(DM_variable.CTRL_MODE))
    if mode != 1:
        control.close()
        sys.exit(f"error: motor is in CTRL_MODE={mode}, not MIT (1).\n"
                 f"  run: .venv/bin/python dm_motor.py mode MIT && "
                 f".venv/bin/python dm_motor.py save-params")

    # ---- controller state ----
    kp, kd = args.kp, args.kd          # hold gains (kd refined in stage 2)
    G, B = 0.0, 0.0                    # gravity fit
    C_VISC, F_COUL = 0.0, 0.0          # friction fit (reported; not fed forward)
    tau_i = 0.0                        # integral term state
    dt = 1.0 / args.rate
    enabled = False

    # ---- primitives ----
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def send(q_ref, dq_ref, tau_ff, kp_use=None, kd_use=None):
        # hard clamps: kd>5 would overflow its 12-bit field in the MIT frame
        # and corrupt the other gains (this caused a runaway once!)
        control.controlMIT(motor,
                           kp=clamp(kp_use if kp_use is not None else kp, 0.0, 500.0),
                           kd=clamp(kd_use if kd_use is not None else kd, 0.0, 5.0),
                           q=q_ref, dq=dq_ref,
                           tau=clamp(tau_ff, -args.tau_max, args.tau_max))

    def integrate(err):
        nonlocal tau_i
        tau_i += args.ki * err * dt
        tau_i = max(-args.i_max, min(args.i_max, tau_i))
        return tau_i

    def fric_ff(err):
        """smooth coulomb friction feedforward: pre-breaks stiction toward the target"""
        if args.fc == 0.0:
            return 0.0
        return args.fc * math.tanh(err / args.fc_eps)

    def grav_ff(q):
        return G * math.sin(q) + B

    def check_guards(q_ref):
        q_meas, dq_meas = float(motor.getPosition()), float(motor.getVelocity())
        if abs(dq_meas) > args.max_vel:
            raise RuntimeError(f"velocity {dq_meas:+.2f} rad/s exceeds --max-vel")
        if abs(q_meas - q_ref) > args.max_track_err:
            raise RuntimeError(f"tracking error {abs(q_meas - q_ref):.2f} rad exceeds --max-track-err")
        return q_meas

    def ramp_to(q_goal, T, kp_use=None, kd_use=None, use_ff=True, use_int=False):
        """smooth cosine ramp from current position to q_goal"""
        nonlocal tau_i
        if use_int:
            tau_i = 0.0
        q_start = float(motor.getPosition())
        loop = Loop(args.rate)
        t = 0.0
        while t < T:
            t = loop.tick()
            q_ref, dq_ref = cosine_ramp(q_start, q_goal, t, T)
            q_meas = check_guards(q_ref)
            ff = grav_ff(q_meas) if use_ff else 0.0
            if use_int:
                ff += integrate(q_ref - q_meas) + fric_ff(q_ref - q_meas)
            send(q_ref, dq_ref, ff, kp_use, kd_use)

    def park_and_disable(reason):
        nonlocal enabled
        print(f"\n[{reason}] parking pendulum before disable...")
        try:
            if args.no_park:
                print("  --no-park given: disabling immediately — arm will drop!")
            elif enabled:
                q_now = float(motor.getPosition())
                q_park = round(q_now / math.pi) * math.pi  # nearest bottom
                if abs(q_now - q_park) > 0.05:
                    ramp_to(q_park, args.ramp_time, use_ff=True)
                    time.sleep(0.3)
        except Exception as e:
            print(f"  park aborted: {e}")
        if enabled:
            control.disable(motor)
            enabled = False
        control.close()
        print("motor disabled, bus closed.")

    try:
        # ============ STAGE 1: gravity + friction fit ============
        control.refresh_motor_status(motor)
        q_home = float(motor.getPosition())
        print(f"current position: {q_home:.4f} rad ({math.degrees(q_home):.1f} deg from 12 o'clock)")

        if args.gravity is not None:
            G, B = args.gravity, args.gravity_offset
            print(f"using given gravity constant G={G:.3f} N*m, B={B:.3f} N*m (stage 1 skipped)")
        else:
            print(f"stage 1: gravity/friction sweep — ±{args.tune_amp} rad, "
                  f"{args.tune_cycles} cycles of {args.tune_period}s, "
                  f"gentle gains kp={args.tune_kp} kd={args.tune_kd}")
            control.enable(motor)
            enabled = True

            # settle gently at home
            loop = Loop(args.rate)
            t = 0.0
            while t < 1.5:
                t = loop.tick()
                send(q_home, 0.0, 0.0, args.tune_kp, args.tune_kd)
                check_guards(q_home)

            # slow sweep with amplitude ease-in (no jump at start)
            qs, dqs, taus = [], [], []
            dq_filt = 0.0
            loop = Loop(args.rate)
            t = 0.0
            T_total = args.tune_period * args.tune_cycles
            w = 2.0 * math.pi / args.tune_period
            ease_T = args.tune_period / 2.0  # ramp amplitude in over first half cycle
            while t < T_total:
                t = loop.tick()
                ease = min(1.0, t / ease_T)
                ease = (1.0 - math.cos(math.pi * ease)) / 2.0  # smooth 0..1
                phase = w * t
                amp = args.tune_amp * ease
                q_ref = q_home + amp * math.sin(phase)
                dq_ref = amp * w * math.cos(phase)
                send(q_ref, dq_ref, 0.0, args.tune_kp, args.tune_kd)
                q_meas = check_guards(q_ref)
                dq_filt = 0.8 * dq_filt + 0.2 * float(motor.getVelocity())
                qs.append(q_meas)
                dqs.append(dq_filt)
                taus.append(float(motor.getTorque()))
                if t % 5.0 < dt:
                    print(f"  sweep {t:5.1f}/{T_total:.0f}s  q={q_meas:+.3f}  "
                          f"tau={taus[-1]:+.3f}", flush=True)

            # return gently to home before fitting
            ramp_to(q_home, 2.0, kp_use=args.tune_kp, kd_use=args.tune_kd, use_ff=False)

            # sanity: the arm must actually have moved, otherwise the fit is garbage
            span = max(qs) - min(qs)
            expected = 2.0 * args.tune_amp * 0.8  # allow for the ease-in portion
            print(f"sweep motion span: {span:.3f} rad (expected >= {expected:.3f})")
            if span < expected:
                park_and_disable("arm did not move during sweep")
                sys.exit("error: sweep too small to fit. The motor could not track it —\n"
                         "  raise --tune-kp (e.g. 60) until the span check passes.")

            # full model fit: tau = G*sin(q) + B + c*dq + fc*sign(dq)
            q_arr, dq_arr, tau_arr = np.array(qs), np.array(dqs), np.array(taus)
            A = np.column_stack([np.sin(q_arr), np.ones(len(q_arr)),
                                 dq_arr, np.sign(dq_arr)])
            coef, *_ = np.linalg.lstsq(A, tau_arr, rcond=None)
            G, B, C_VISC, F_COUL = [float(c) for c in coef]
            pred = A @ coef
            ss_res = float(np.sum((tau_arr - pred) ** 2))
            ss_tot = float(np.sum((tau_arr - np.mean(tau_arr)) ** 2)) or 1e-9
            r2 = 1.0 - ss_res / ss_tot
            print(f"stage 1 fit:  G={G:+.3f} N*m  B={B:+.3f} N*m  "
                  f"viscous c={C_VISC:+.3f} N*m*s  coulomb fc={F_COUL:+.3f} N*m  "
                  f"(R²={r2:.3f}, n={len(q_arr)})")
            # NEVER apply a bad fit as feedforward — that is how runaways happen
            if r2 < 0.5 or abs(G) > 100.0:
                park_and_disable(f"implausible gravity fit (G={G:+.2f}, R²={r2:.2f})")
                sys.exit("error: fit not trustworthy; refusing to use it as feedforward.\n"
                         "  try: --tune-kp 60, --tune-amp 0.8, --tune-period 40")
            if r2 < 0.8:
                print("WARNING: mediocre fit — feedforward may be inaccurate. "
                      "Consider --tune-period 40 or --tune-amp 0.8")
            print(f"gravity feedforward: tau_ff(q) = {G:+.3f}*sin(q) {B:+.3f}")

        if not enabled:
            control.enable(motor)
            enabled = True

        # ============ STAGE 2: push-recovery tuning ============
        ramp_to(args.target, args.ramp_time)
        q_goal = args.target
        push_dir = 1.0 if q_goal >= 0 else -1.0  # push toward the bottom (gravity side)

        if args.no_return_tune:
            print(f"stage 2 skipped; holding with kp={kp} kd={kd} ki={args.ki}")
        else:
            print(f"stage 2: push-recovery tuning (±{args.push_amp} rad steps, auto kd)...")
            best = (float("inf"), kd)  # (score, kd)
            for attempt in range(6):
                # displace: slow ramp out by push_amp, brief hold
                ramp_to(q_goal + push_dir * args.push_amp, 1.2, use_int=True)
                loop = Loop(args.rate)
                t = 0.0
                while t < 0.8:
                    t = loop.tick()
                    q_meas = check_guards(q_goal + push_dir * args.push_amp)
                    err = q_goal + push_dir * args.push_amp - q_meas
                    send(q_goal + push_dir * args.push_amp, 0.0,
                         grav_ff(q_meas) + integrate(err) + fric_ff(err))

                # release: step reference back to goal, record the transient
                errs = []
                loop = Loop(args.rate)
                t = 0.0
                while t < 3.0:
                    t = loop.tick()
                    q_meas = check_guards(q_goal)
                    err = q_goal - q_meas
                    send(q_goal, 0.0, grav_ff(q_meas) + integrate(err) + fric_ff(err))
                    errs.append((t, q_meas - q_goal))

                err_arr = np.array([e for _, e in errs])
                overshoot = max(0.0, -push_dir * err_arr.min()) / args.push_amp
                # settling: last time |err| outside ±0.02 rad band
                outside = [tt for tt, e in errs if abs(e) > 0.02]
                settle = (outside[-1] if outside else 0.0)
                score = overshoot + settle / 3.0
                print(f"  attempt {attempt + 1}: kd={kd:.2f}  overshoot={overshoot * 100:.0f}%  "
                      f"settle={settle:.2f}s", flush=True)
                if score < best[0]:
                    best = (score, kd)
                if overshoot > 0.15:
                    kd = min(args.kd_max, kd * 1.6)
                elif overshoot > 0.05:
                    kd = min(args.kd_max, kd * 1.25)
                elif settle > 2.0:
                    kd = max(args.kd_min, kd * 0.8)
                else:
                    print(f"  converged: kd={kd:.2f}")
                    break
                if kd in (args.kd_min, args.kd_max):
                    break
            if not (overshoot <= 0.05 and settle <= 2.0):
                kd = best[1]
                print(f"  using best found kd={kd:.2f}")
            print(f"stage 2 result: hold gains kp={kp} kd={kd:.2f} ki={args.ki}")

        # ============ STAGE 3: HOLD ============
        tau_i = 0.0
        print("holding target. push it if you like — Ctrl-C to park and exit.")
        t_last = 0.0
        loop = Loop(args.rate)
        while True:
            t = loop.tick()
            q_meas = check_guards(q_goal)
            err = q_goal - q_meas
            send(q_goal, 0.0, grav_ff(q_meas) + integrate(err) + fric_ff(err))
            if t - t_last >= 0.5:
                t_last = t
                print(f"  q={q_meas:+.4f} rad  dq={motor.getVelocity():+.3f} rad/s  "
                      f"tau={motor.getTorque():+.3f} N*m  i={tau_i:+.2f} N*m", flush=True)

    except KeyboardInterrupt:
        park_and_disable("Ctrl-C")
    except Exception as e:
        park_and_disable(f"abort: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
