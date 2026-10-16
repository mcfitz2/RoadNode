"""Thresholds of the car-battery low-voltage cutoff (circuit.py, UVLO block).

VIN -> R1 -> UV_TAP -> R2 -> GND (C filter) on the inverting input. VREF (REF3325,
2.5 V) -> RA -> UV_REF -> RB -> GND on the non-inverting input, RH from the
open-drain output (RPU pull-up to the 3.3 V rail) back to UV_REF. Output high =
battery low = buck EN pulled low.

Prints nominal thresholds, the worst case over 1% resistors, a 0.15% VREF and a
3% pull-up rail (HT7533), and how long a cranking dip lasts before the cutoff trips.
"""
import itertools
import math

R1, R2, RA, RB, RH, RPU, VR, V3, C = 3e6, 750e3, 510e3, 10e6, 20e6, 1e6, 2.5, 3.3, 10e-6


def thresholds(r1, r2, ra, rb, rh, rpu, vr, v3):
    k = (r1 + r2) / r2
    ref_hi = (vr / ra + v3 / (rh + rpu)) / (1 / ra + 1 / rb + 1 / (rh + rpu))  # output released
    ref_lo = (vr / ra) / (1 / ra + 1 / rb + 1 / rh)  # output low
    return ref_hi * k, ref_lo * k  # turn on, turn off


on, off = thresholds(R1, R2, RA, RB, RH, RPU, VR, V3)
print(f"nominal: on above {on:.2f} V, off below {off:.2f} V")
t = (0.99, 1.01)
cases = [thresholds(R1 * a, R2 * b, RA * c, RB * d, RH * e, RPU, VR * f, V3 * g)
         for a, b, c, d, e, f, g in itertools.product(t, t, t, t, t, (0.9985, 1.0015), (0.97, 1.03))]
print(f"worst case: on {min(c[0] for c in cases):.2f}..{max(c[0] for c in cases):.2f} V, "
      f"off {min(c[1] for c in cases):.2f}..{max(c[1] for c in cases):.2f} V")
tau = C * R1 * R2 / (R1 + R2)
print(f"filter tau {tau:.1f} s")
for rest, crank in ((12.6, 9.5), (12.0, 9.5), (12.6, 7.0)):
    t_trip = tau * math.log((rest - crank) / (off - crank))
    print(f"  rest {rest} V, crank dip to {crank} V: trips after {t_trip:.1f} s")
print(f"divider currents at 12.6 V: tap {12.6 / (R1 + R2) * 1e6:.1f} uA, ref {VR / (RA + RB) * 1e6:.2f} uA")
