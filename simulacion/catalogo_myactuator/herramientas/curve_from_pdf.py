"""Reconstruye la tabla de la curva caracteristica (T,N,Pout,U,I,Pin,Eff) a partir de pdftotext -bbox."""
import re, subprocess, sys, html
def words(pdf, page=1):
    out = subprocess.run(["pdftotext", "-bbox", "-f", str(page), "-l", str(page), pdf, "-"],
                         capture_output=True, text=True).stdout
    ws = []
    for m in re.finditer(r'<word xMin="([\d.]+)" yMin="([\d.]+)" xMax="([\d.]+)" yMax="([\d.]+)">(.*?)</word>', out):
        ws.append((float(m[1]), float(m[2]), float(m[3]), float(m[4]), html.unescape(m[5])))
    return ws
def table(pdf, page=1):
    ws = words(pdf, page)
    heads = [w for w in ws if w[4].replace(' ', '').startswith(('T(N','T(mN'))]
    if not heads: return None, "sin encabezado T(N.m)"
    h = max(heads, key=lambda w: w[0])            # tabla de datos = la mas a la derecha
    # encabezados de la misma fila
    row = sorted([w for w in ws if abs(w[1]-h[1]) < 3 and w[0] >= h[0]-2], key=lambda w: w[0])
    cols = [w[0] for w in row][:7]
    x0, x1 = h[0]-15, row[min(6, len(row)-1)][2] + 15
    num = re.compile(r'^-?\d+(\.\d+)?$')
    cand = [w for w in ws if w[1] > h[3] and x0 <= w[0] <= x1 and num.match(w[4])]
    cand.sort(key=lambda w: (w[1], w[0]))
    rows, cur, y = [], [], None
    for w in cand:
        if y is None or abs(w[1]-y) < 2.5: cur.append(w); y = w[1] if y is None else y
        else: rows.append(cur); cur, y = [w], w[1]
    if cur: rows.append(cur)
    data = []
    for r in rows:
        r.sort(key=lambda w: w[0])
        vals = [None]*7
        for w in r:
            xc = (w[0]+w[2])/2
            k = min(range(len(cols)), key=lambda i: abs(((row[i][0]+row[i][2])/2) - xc))
            vals[k] = float(w[4])
        if sum(v is not None for v in vals) >= 6: data.append(vals)
    return data, [w[4] for w in row]
if __name__ == "__main__":
    d, hdr = table(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 1)
    print(hdr); 
    for r in d or []: print(r)
