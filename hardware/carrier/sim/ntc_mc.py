"""Monte Carlo of the NTC gate trip points: 1% resistors, NTC R25 1% / B 1%, comparator offset 10 mV max, VREF 2%, V5 3%."""
import math, random
def trips(p):
    rntc = lambda t: p['r25']*math.exp(p['b']*(1/(t+273.15)-1/298.15))
    vt = lambda t: p['vref']*rntc(t)/(p['rtop']+rntc(t))
    def tap(r1, r2, rh, vfb, rpu):
        g = 1/r1 + 1/r2 + 1/(rh+rpu)
        return (p['vref']/r1 + vfb/(rh+rpu))/g
    def inv(v):
        lo, hi = -60, 120
        for _ in range(50):
            m = (lo+hi)/2; lo, hi = (m, hi) if vt(m) > v else (lo, m)
        return m
    a, b = p['a'], p['b_']
    return (inv(tap(*a, p['v5'], p['rpu'])+p['oa']), inv(tap(*a, 0, p['rpu'])+p['oa']),
            inv(tap(*b, 0, p['rpu'])+p['ob']), inv(tap(*b, p['v5'], p['rpu'])+p['ob']))
t = lambda v, pct: v*(1+random.uniform(-pct, pct)/100)
res = []
random.seed(1)
for _ in range(20000):
    p = dict(r25=t(100e3,1), b=t(3950,1), rtop=t(100e3,1), vref=t(3.3,2), v5=t(5.17,3), rpu=t(1e6,1),
             a=(t(100e3,1), t(300e3,1), t(5.1e6,1)), b_=(t(470e3,1), t(220e3,1), t(10e6,1)),
             oa=random.uniform(-10e-3,10e-3), ob=random.uniform(-10e-3,10e-3))
    res.append(trips(p))
for i, n in enumerate(['cold off', 'cold on', 'hot off', 'hot on']):
    v = sorted(r[i] for r in res)
    print(f"{n:9} min {v[0]:6.2f}  median {v[len(v)//2]:6.2f}  max {v[-1]:6.2f} C")
