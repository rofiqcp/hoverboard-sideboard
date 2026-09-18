#!/usr/bin/env python3
"""Allan-deviation analysis for long stationary IMU logs."""
import argparse,csv,math
import numpy as np
COLS=["gx_rads","gy_rads","gz_rads","ax_mps2","ay_mps2","az_mps2"]

def load(path):
    host=[]; board=[]; data={k:[] for k in COLS}; temp=[]
    with open(path,newline="") as f:
        for r in csv.DictReader(f):
            try:
                host.append(float(r["host_s"])); board.append(int(r["board_us"]) & 0xffffffff); temp.append(float(r["temp_c"]))
                for k in COLS:data[k].append(float(r[k]))
            except (KeyError,ValueError): pass
    if len(host)<1000: raise SystemExit("ERROR: log terlalu pendek (<1000 sample)")
    # Gunakan clock MCU, bukan median host inter-arrival. Serial dapat burst sehingga
    # median host_s menghasilkan sample-rate palsu. board_us di-unwrap modulo 2^32.
    total_us=0
    for a,b in zip(board,board[1:]): total_us += (b-a) & 0xffffffff
    if total_us>0:
        duration=total_us*1.0e-6; fs=(len(board)-1)/duration
    else:
        duration=host[-1]-host[0]; fs=(len(host)-1)/duration
    if not (5.0 <= fs <= 1000.0): raise SystemExit(f"ERROR: sample-rate log tidak masuk akal: {fs:.3f} Hz")
    return fs,np.asarray(temp),{k:np.asarray(v) for k,v in data.items()}

def allan(x,fs):
    n=len(x); maxm=max(2,n//20); ms=np.unique(np.logspace(0,math.log10(maxm),45).astype(int)); out=[]
    c=np.concatenate(([0.0],np.cumsum(x,dtype=float)))
    for m in ms:
        k=n//m
        if k<3: continue
        means=(c[m:m*k+1:m]-c[0:m*k:m])/m
        av=math.sqrt(0.5*float(np.mean(np.diff(means)**2))); out.append((m/fs,av))
    return out

def estimate(curve):
    tau=np.array([p[0] for p in curve]); av=np.array([p[1] for p in curve]);
    i1=int(np.argmin(np.abs(np.log(tau)-math.log(1.0)))); white=av[i1]*math.sqrt(tau[i1])
    bias=float(np.min(av))/0.664 if len(av) else float('nan')
    slopes=np.diff(np.log(av))/np.diff(np.log(tau)); candidates=np.where(tau[:-1]>10.0)[0]
    if len(candidates):
        j=int(candidates[np.argmin(np.abs(slopes[candidates]-0.5))]); rw=av[j]*math.sqrt(3.0/tau[j]); slope=float(slopes[j])
    else: rw=float('nan'); slope=float('nan')
    return white,bias,rw,slope

def main():
    ap=argparse.ArgumentParser(description="Allan deviation dari log stationary panjang")
    ap.add_argument("file"); ap.add_argument("--curve-out",default=None); a=ap.parse_args()
    fs,temp,d=load(a.file); dur=len(temp)/fs
    print(f"samples={len(temp)} fs={fs:.3f}Hz duration={dur/60:.1f}min temp={temp.min():.2f}..{temp.max():.2f}C span={np.ptp(temp):.2f}C")
    if np.ptp(temp)>2.0: print("WARNING: temperature span >2C; bias/long-tau Allan tercampur thermal drift")
    curves={}; est={}
    for k in COLS:
        curves[k]=allan(d[k],fs); est[k]=estimate(curves[k]); w,b,rw,sl=est[k]
        print(f"{k}: white_density={w:.6g} bias_instability~={b:.6g} random_walk~={rw:.6g} slope={sl:.3f}")
    gnoise=max(est[k][0] for k in COLS[:3])*1.2; anoise=max(est[k][0] for k in COLS[3:])*1.2
    gb=[est[k][2] for k in COLS[:3] if math.isfinite(est[k][2])]; ab=[est[k][2] for k in COLS[3:] if math.isfinite(est[k][2])]
    gbias=max(gb)*1.2 if gb else None; abias=max(ab)*1.2 if ab else None
    print(f"recommended gyro_noise~{gnoise:.6g}, accel_process_noise~{anoise:.6g}")
    if dur>=600 and gbias is not None and abias is not None: print(f"candidate bias_walk gyro~{gbias:.6g}, accel~{abias:.6g} (verify curve before applying)")
    else: print("bias_walk: log >=10 min diperlukan; jangan ubah dari log pendek")
    if a.curve_out:
        with open(a.curve_out,"w",newline="") as f:
            w=csv.writer(f); w.writerow(["signal","tau_s","allan_dev"])
            for k,c in curves.items():
                for tau,v in c:w.writerow([k,tau,v])
        print("curve saved:",a.curve_out)
    return 0
if __name__=="__main__": raise SystemExit(main())
