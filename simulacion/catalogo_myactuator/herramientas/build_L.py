"""Tabla final de una grafica L digitalizada: N, T, Pout, I, Eff muestreados directamente de cada curva."""
import json, sys, numpy as np
def smooth(pts, win=4):
    p = np.asarray(pts, float); p = p[np.argsort(p[:, 0])]
    y = np.array([np.median(p[max(0, i-win):i+win+1, 1]) for i in range(len(p))])
    return p[:, 0], y
def endfit(x, y, lo, hi):
    k = (x >= lo) & (x <= hi); m, b = np.polyfit(x[k], y[k], 1); return m, b
d = json.load(open(sys.argv[1])); out = sys.argv[2]
c = {k: smooth(v) for k, v in d["curves"].items()}
xT, yT = c["T"]; span = xT.max()
m, b = endfit(xT, yT, 0.7*span, span); n_end = -b/m          # donde la curva de par llega a 0
grid = np.linspace(0, n_end, 21)
def sample(name, n):
    x, y = c[name]
    if n < x.min():                                          # extrapola al inicio con el primer 15 %
        mm, bb = endfit(x, y, x.min(), x.min() + 0.15*span); return mm*n + bb
    if n > x.max():
        mm, bb = endfit(x, y, x.max() - 0.15*span, x.max()); return mm*n + bb
    return float(np.interp(n, x, y))
rows = []
for n in grid:
    T = max(sample("T", n), 0)/1000; I = max(sample("I", n), 0)
    P = 0.0 if n in (grid[0], grid[-1]) else max(sample("Pout", n), 0)
    E = 0.0 if n == grid[0] else max(sample("Eff", n), 0)*100
    rows.append((T, n, P, I, E))
with open(out, "w") as f:
    f.write("torque_Nm,speed_rpm,output_power_W,current_A,efficiency_pct\n")
    for T, n, P, I, E in rows: f.write(f"{T:.4f},{n:.0f},{P:.2f},{I:.3f},{E:.1f}\n")
err = [abs(P - T*n*2*np.pi/60)/P for T, n, P, I, E in rows[3:-3]]
print(f"{out}: n_end={n_end:.0f} rpm, T(0)={rows[0][0]:.3f} N.m, I(0)={rows[0][3]:.2f} A, "
      f"|Pout curva - T*w|/Pout mediana {100*np.median(err):.1f}% max {100*max(err):.1f}%")
