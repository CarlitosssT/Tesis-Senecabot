"""Compara la transcripcion manual con el OCR y verifica la consistencia fisica con tolerancia de redondeo."""
import ast, math, sys
man = [[float(x) for x in l.split()] for l in open(sys.argv[1]) if l.strip()]
ocr = [ast.literal_eval(l.split('#')[0]) for l in open(sys.argv[2]) if l.strip()] if len(sys.argv) > 2 else []
k = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
w = 2*math.pi/60
print(f"manual {len(man)} filas, ocr {len(ocr)} filas")
for i, r in enumerate(man):
    T, N, Po, U, I, Pi, E = r
    tolP = (0.005*k*N + T*k*0.5)*w + 0.006 + 0.02*Po*0   # redondeo de T y N
    tolI = 0.0005*U + 0.005*I + 0.006
    msg = []
    if abs(T*k*N*w - Po) > tolP: msg.append(f"Pout calc {T*k*N*w:.2f}")
    if abs(U*I - Pi) > tolI: msg.append(f"Pin calc {U*I:.2f}")
    if Pi and abs(100*Po/Pi - E) > 0.15: msg.append(f"Eff calc {100*Po/Pi:.1f}")
    o = next((x for x in ocr if abs(x[0]-T) < 1e-9 and abs(x[1]-N) < 1e-9), None) if ocr else None
    if ocr:
        cand = [x for x in ocr if sum(abs(a-b) < 1e-9 for a, b in zip(x, r)) >= 4]
        if not any(all(abs(a-b) < 1e-9 for a, b in zip(x, r)) for x in cand):
            best = max(ocr, key=lambda x: sum(abs(a-b) < 1e-9 for a, b in zip(x, r)))
            msg.append(f"OCR difiere: {best}")
    if msg: print(i, r, "|", "; ".join(msg))
