import json, sys, numpy as np
def robust_line(x, y, it=5):
    w = np.ones_like(x)
    for _ in range(it):
        A = np.vstack([x, np.ones_like(x)]).T * w[:, None]
        m, b = np.linalg.lstsq(A, y*w, rcond=None)[0]
        r = y - (m*x + b); s = np.median(abs(r)) * 1.5 + 1e-9
        w = (abs(r) < 3*s).astype(float)
    return m, b, float(np.sqrt(np.mean(r[w > 0]**2)))
def interp(pts, x):
    p = np.array(pts); o = np.argsort(p[:, 0]); return np.interp(x, p[o, 0], p[o, 1])
if __name__ == "__main__":
    d = json.load(open(sys.argv[1])); U = float(sys.argv[2]); sheet = json.loads(sys.argv[3])
    c = {k: np.array(v) for k, v in d["curves"].items()}
    # solo puntos de una sola linea (sin mezcla) para el ajuste
    T = c["T"]; I = c["I"]
    mT, bT, rT = robust_line(T[:, 0], T[:, 1]); mI, bI, rI = robust_line(I[:, 0], I[:, 1])
    n0 = -bT/mT
    print(f"T(n)= {bT:.1f} {mT:+.4f} n  [mN.m] rms {rT:.1f}; stall T = {bT/1000:.3f} N.m (ficha {sheet['Tmax']}); n sin carga = {n0:.0f} rpm (ficha {sheet['nmax']})")
    print(f"I(n)= {bI:.3f} {mI:+.6f} n [A] rms {rI:.3f}; I(0) = {bI:.2f} A (ficha max {sheet['Imax']})")
    ns = np.linspace(0, n0, 9)[1:-1]
    for n in ns:
        Tn = (bT + mT*n)/1000; In = bI + mI*n; Pc = Tn*n*2*np.pi/60
        Pd = interp(c["Pout"], n); Ed = interp(c["Eff"], n)*100
        print(f"  n={n:6.0f}  P(T*w)={Pc:6.2f}  P(curva)={Pd:6.2f}  Eff(calc)={100*Pc/(U*In):5.1f}  Eff(curva)={Ed:5.1f}")
