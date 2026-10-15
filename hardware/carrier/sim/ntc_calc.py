"""Pick E96 values for the NTC charge-gate thresholds (see ntc_gate.cir)."""
import math, itertools
VREF, V5, RPU, RTOP, R25, B = 3.3, 5.17, 1e6, 100e3, 100e3, 3950
E96 = [1.00,1.02,1.05,1.07,1.10,1.13,1.15,1.18,1.21,1.24,1.27,1.30,1.33,1.37,1.40,1.43,1.47,1.50,1.54,1.58,1.62,1.65,1.69,1.74,1.78,1.82,1.87,1.91,1.96,2.00,2.05,2.10,2.15,2.21,2.26,2.32,2.37,2.43,2.49,2.55,2.61,2.67,2.74,2.80,2.87,2.94,3.01,3.09,3.16,3.24,3.32,3.40,3.48,3.57,3.65,3.74,3.83,3.92,4.02,4.12,4.22,4.32,4.42,4.53,4.64,4.75,4.87,4.99,5.11,5.23,5.36,5.49,5.62,5.76,5.90,6.04,6.19,6.34,6.49,6.65,6.81,6.98,7.15,7.32,7.50,7.68,7.87,8.06,8.25,8.45,8.66,8.87,9.09,9.31,9.53,9.76]
vals = [m*10**e for e in (4,5,6) for m in E96]
rntc = lambda t: R25*math.exp(B*(1/(t+273.15)-1/298.15))
vt = lambda t: VREF*rntc(t)/(RTOP+rntc(t))
def tap(r1, r2, rh, vfb):  # divider VREF-r1-node-r2-GND, rh from node to vfb (vfb=V5 via RPU, or 0)
    g = 1/r1 + 1/r2 + 1/(rh + (RPU if vfb else 0))
    return (VREF/r1 + vfb/(rh + (RPU if vfb else 0)))/g
def inv(v):  # temperature where vt == v
    lo, hi = -40, 100
    for _ in range(60):
        m = (lo+hi)/2
        lo, hi = (m, hi) if vt(m) > v else (lo, m)
    return m
best = {}
# A (cold): + = HA, - = T. out high (OK) feeds back: trip when T rises to HA(out high); recover when T falls to HA(out low)
# B (hot):  + = HB, - = T. out low normally; trip (T falls) at HB(out low); recover at HB(out high)
for name, (ttrip, trec, trip_hi) in {'A': (1, 4, True), 'B': (44, 41, False)}.items():
    cands = []
    for r1, r2 in itertools.product([v for v in vals if 1e5 <= v <= 1e6], repeat=2):
        for rh in [v for v in vals if 1e6 <= v <= 1e7] + [v*10 for v in vals if 1e6 <= v <= 2e6]:
            hi, lo = tap(r1, r2, rh, V5), tap(r1, r2, rh, 0)
            t_trip, t_rec = (inv(hi), inv(lo)) if trip_hi else (inv(lo), inv(hi))
            cands.append((abs(t_trip-ttrip)+abs(t_rec-trec), r1, r2, rh, t_trip, t_rec))
    best[name] = min(cands)
    e, r1, r2, rh, a, b = best[name]
    print(f"{name}: R1={r1/1e3:g}k R2={r2/1e3:g}k Rh={rh/1e6:g}M -> trip {a:.2f} C, recover {b:.2f} C")
print(f"T node: 0C {vt(0):.3f} V, 25C {vt(25):.3f} V, 45C {vt(45):.3f} V, -40C {vt(-40):.3f} V, 85C {vt(85):.3f} V")
