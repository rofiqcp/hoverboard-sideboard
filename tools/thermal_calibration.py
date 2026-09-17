#!/usr/bin/env python3
"""Fit linear IMU thermal slopes from multiple stationary temperature plateaus."""
import argparse,csv,math,statistics,subprocess,sys
G=9.80665
AX=("ax_mps2","ay_mps2","az_mps2"); GX=("gx_rads","gy_rads","gz_rads")

def load_plateau(path):
    rows=[]
    with open(path,newline="") as f:
        for r in csv.DictReader(f):
            try:
                v={k:float(r[k]) for k in ("temp_c",)+AX+GX}
            except (ValueError,KeyError): continue
            an=math.sqrt(sum(v[k]*v[k] for k in AX)); gn=math.sqrt(sum(v[k]*v[k] for k in GX))*180/math.pi
            if abs(an/G-1.0)<=0.12 and gn<=3.0: rows.append(v)
    if len(rows)<100: raise ValueError(f"{path}: sample stationary valid <100")
    mean=lambda k:statistics.fmean(r[k] for r in rows)
    return {"path":path,"n":len(rows),"temp":mean("temp_c"),"a":[mean(k) for k in AX],"g":[mean(k) for k in GX]}

def fit(xs,ys):
    xm=statistics.fmean(xs); ym=statistics.fmean(ys); den=sum((x-xm)**2 for x in xs)
    if den<=1e-9: raise ValueError("temperature spread nol")
    m=sum((x-xm)*(y-ym) for x,y in zip(xs,ys))/den; b=ym-m*xm
    ss=sum((y-ym)**2 for y in ys); se=sum((y-(m*x+b))**2 for x,y in zip(xs,ys)); r2=1-se/ss if ss>1e-15 else 1.0
    return m,b,r2

def main():
    ap=argparse.ArgumentParser(description="Fit thermal slope; gunakan >=3 plateau dengan board orientasi tetap")
    ap.add_argument("files",nargs="+"); ap.add_argument("--apply",action="store_true")
    a=ap.parse_args(); ps=[load_plateau(x) for x in a.files]
    if len(ps)<3: raise SystemExit("ERROR: minimal 3 plateau temperatur")
    temps=[p["temp"] for p in ps]; span=max(temps)-min(temps)
    if span<8.0: raise SystemExit(f"ERROR: temperature span {span:.2f}C < 8C")
    # Orientation must stay fixed: compare mean accel unit-vector against first plateau.
    ref=ps[0]["a"]; rn=math.sqrt(sum(x*x for x in ref)); ref=[x/rn for x in ref]
    for p in ps[1:]:
        n=math.sqrt(sum(x*x for x in p["a"])); u=[x/n for x in p["a"]]; dot=sum(x*y for x,y in zip(ref,u))
        if dot<0.995: raise SystemExit(f"ERROR: orientasi berubah pada {p['path']} (dot={dot:.6f})")
    gs=[]; ac=[]
    print("Plateau:")
    for p in ps: print(f"  {p['path']}: T={p['temp']:.2f}C n={p['n']}")
    for i in range(3):
        m,b,r2=fit(temps,[p["g"][i] for p in ps]); gs.append(m); print(f"gyro{i}: {m*180/math.pi:+.6f} deg/s/C R2={r2:.4f}")
    for i in range(3):
        m,b,r2=fit(temps,[p["a"][i] for p in ps]); ac.append(m); print(f"accel{i}: {m:+.6f} m/s2/C R2={r2:.4f}")
    gd=[x*180/math.pi for x in gs]
    cmd=[sys.executable,"tools/configure_imu.py","thermal",*(f"{x:.9g}" for x in gd),*(f"{x:.9g}" for x in ac)]
    print("Command:"," ".join(cmd))
    if a.apply:
        print("Applying thermal slopes..."); return subprocess.call(cmd)
    return 0
if __name__=="__main__": raise SystemExit(main())
