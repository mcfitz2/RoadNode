"""Thresholds of the engine-running wake comparator (circuit.py, PROJ-595).

VIN -> R1 -> V_TAP -> R2 -> GND, 10M from the open-drain output (1M pull-up
to V3REF) back to V_TAP, V3REF/300k/750k reference on the inverting input.
Prints nominal thresholds and the worst case over 1% resistors and a 2% V3REF.
"""
import itertools

R1, R2, RA, RB, RF, RPU, V3 = 1e6, 220e3, 300e3, 750e3, 10e6, 1e6, 3.3


def thresholds(r1, r2, ra, rb, rf, rpu, v3):
    vref = v3 * rb / (ra + rb)
    # KCL at V_TAP = vref: (Vin - vref)/r1 = vref/r2 + (vref - vout)/(rf + rsrc)
    rise = vref + r1 * (vref / r2 + vref / rf)
    fall = vref + r1 * (vref / r2 + (vref - v3) / (rf + rpu))
    return rise, fall


rise, fall = thresholds(R1, R2, RA, RB, RF, RPU, V3)
print(f"nominal: on {rise:.2f} V, off {fall:.2f} V, tap at 14.8 V {14.8 * R2 / (R1 + R2):.2f} V")
cases = [thresholds(R1 * a, R2 * b, RA * c, RB * d, RF, RPU, V3 * e)
         for a, b, c, d, e in itertools.product((0.99, 1.01), (0.99, 1.01), (0.99, 1.01), (0.99, 1.01), (0.98, 1.02))]
print(f"worst case: on {min(c[0] for c in cases):.2f}..{max(c[0] for c in cases):.2f} V, "
      f"off {min(c[1] for c in cases):.2f}..{max(c[1] for c in cases):.2f} V")
