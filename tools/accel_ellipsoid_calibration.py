#!/usr/bin/env python3
"""Multi-orientation accelerometer ellipsoid fit (host-side, 12+ stationary poses)."""
import argparse,csv,math,statistics,subprocess,sys
import numpy as np
from scipy.optimize import least_squares
G=9.80665
COLS=("ax_mps2","ay_mps2","az_mps2")

def plateau(path):
    rows=[]; temps=[]
    with open(path,newline="") as f:
        for r in csv.DictReader(f):
            try:
                a=np.array([float(r[k]) for k in COLS]); g=np.array([float(r[k]) for k in ("gx_rads","gy_rads","gz_rads")]); t=float(r["temp_c"])
            except (KeyError,ValueError):continue
            if abs(np.linalg.norm(a)/G-1.0)<=0.15 and np.linalg.norm(g)*180/math.pi<=3.0:
                rows.append(a);temps.append(t)
    if len(rows)<100:raise ValueError(f"{path}: <100 stationary samples")
    return np.mean(rows,axis=0),statistics.fmean(temps),len(rows)

def matrix_from_p(p):
    # SPD correction matrix M=L*L.T; exp diagonal guarantees positive definite.
    L=np.array([[math.exp(p[3]),0,0],[p[4],math.exp(p[5]),0],[p[6],p[7],math.exp(p[8])]],float)
    return L@L.T

def residual(p,X):
    b=p[:3];M=matrix_from_p(p);return np.linalg.norm((X-b)@M.T,axis=1)-G

def main():
    ap=argparse.ArgumentParser(description="Fit symmetric 3x3 accel correction from >=12 stationary orientations")
    ap.add_argument("files",nargs="+");ap.add_argument("--apply",action="store_true");a=ap.parse_args()
    if len(a.files)<12:raise SystemExit("ERROR: minimal 12 orientation plateau; 18-24 lebih baik")
    ps=[plateau(f) for f in a.files];X=np.vstack([x[0] for x in ps]);temps=np.array([x[1] for x in ps])
    if np.ptp(temps)>5.0:raise SystemExit(f"ERROR: temperature span {np.ptp(temps):.2f}C >5C; lakukan pada suhu relatif sama / thermal correction dulu")
    U=X/np.linalg.norm(X,axis=1)[:,None];cov=U.T@U/len(U);coverage=float(np.min(np.linalg.eigvalsh(cov)))
    if coverage<0.08:raise SystemExit(f"ERROR: orientation coverage buruk (min eigen={coverage:.4f}); sebar pose ke seluruh sphere")
    p0=np.zeros(9);p0[:3]=np.mean(X,axis=0)*0.02
    sol=least_squares(residual,p0,args=(X,),method="trf",max_nfev=3000,xtol=1e-12,ftol=1e-12,gtol=1e-12)
    if not sol.success:raise SystemExit("ERROR: fit tidak converge: "+sol.message)
    b=sol.x[:3];M=matrix_from_p(sol.x);err=residual(sol.x,X);rms=float(np.sqrt(np.mean(err*err)));mx=float(np.max(np.abs(err)))
    cond=float(np.linalg.cond(M));det=float(np.linalg.det(M))
    print(f"poses={len(X)} coverage={coverage:.4f} temp_span={np.ptp(temps):.2f}C")
    print(f"RMS={rms/G:.5f}g max={mx/G:.5f}g cond2={cond:.3f} det={det:.4f}")
    print("offset m/s2:",*(f"{x:+.8f}" for x in b));print("matrix:")
    for r in M:print(" "," ".join(f"{x:+.9f}" for x in r))
    if rms>0.04*G or mx>0.08*G or cond>3.0 or not (0.2<abs(det)<5.0):raise SystemExit("ERROR: quality gate fit gagal; jangan apply")
    vals=[*b,*M.reshape(-1)];cmd=[sys.executable,"tools/configure_imu.py","accel-cal",*(f"{x:.10g}" for x in vals)]
    print("Command:"," ".join(cmd))
    if a.apply:return subprocess.call(cmd)
    return 0
if __name__=="__main__":raise SystemExit(main())
