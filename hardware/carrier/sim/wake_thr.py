"""Thresholds of the engine-running wake comparator (circuit.py, PROJ-595).

VIN -> R1 -> V_TAP -> R2 -> GND, 10M from the open-drain output (1M pull-up
to the 3.3 V rail) back to V_TAP, VREF (REF3325, 2.5 V)/47k/750k reference on
the inverting input. Prints nominal thresholds and the worst case over 1%
resistors, a 0.15% VREF and a 3% pull-up rail (HT7533).
"""
import itertools

R1, R2, RA, RB, RF, RPU, VR, V3 = 1e6, 220e3, 47e3, 750e3, 10e6, 1e6, 2.5, 3.3


def thresholds(r1, r2, ra, rb, rf, rpu, vr, v3):
    vref = vr * rb / (ra + rb)
    # KCL at V_TAP = vref: (Vin - vref)/r1 = vref/r2 + (vref - vout)/(rf + rsrc)
    rise = vref + r1 * (vref / r2 + vref / rf)
    fall = vref + r1 * (vref / r2 + (vref - v3) / (rf + rpu))
    return rise, fall


rise, fall = thresholds(R1, R2, RA, RB, RF, RPU, VR, V3)
print(f"nominal: on {rise:.2f} V, off {fall:.2f} V, tap at 14.8 V {14.8 * R2 / (R1 + R2):.2f} V")
t = (0.99, 1.01)
cases = [thresholds(R1 * a, R2 * b, RA * c, RB * d, RF, RPU, VR * e, V3 * f)
         for a, b, c, d, e, f in itertools.product(t, t, t, t, (0.9985, 1.0015), (0.97, 1.03))]
print(f"worst case: on {min(c[0] for c in cases):.2f}..{max(c[0] for c in cases):.2f} V, "
      f"off {min(c[1] for c in cases):.2f}..{max(c[1] for c in cases):.2f} V")
