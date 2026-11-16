# Single-axis (elevation) 1 kHz time simulation of the LOS_Controller_1kHz law (akgoz_build_controller.m); sweeps used for dob_cutoff / rate_wc_lead.
# Simplified: 1-DOF inertial LOS, friction coupling to base rate, gravity imbalance, ARW noise. Plain Python 3.
import math, random
def run(dob, lead, T=6.0, FWC=None, FD=40):
    WC=2*math.pi*(FWC or (40 if lead else 25))
    J=0.003897; kt=0.168; imax=2.65; fs=1000; dt=1/fs; sub=20; h=dt/sub  # J: akgoz_inertia.m J.el (GEOMETRY=cad)
    wc=WC; wi=wc/4; a_l=1-math.exp(-2*math.pi*150/fs); a_d=1-math.exp(-2*math.pi*FD/fs)
    ph=math.radians(30); ar=(1+math.sin(ph))/(1-math.sin(ph)); wz=wc/math.sqrt(ar); wp=wc*math.sqrt(ar); kl=1/math.sqrt(ar); A=2*fs
    b0=kl*(A/wz+1); b1=kl*(1-A/wz); a0=A/wp+1; a1=1-A/wp
    ag=1-math.exp(-2*math.pi*262/fs); nd=2; buf=[0.0]*8; yg=0.0
    Fc,Fs,vs,b,eps=0.008,0.012,0.01,0.002,1e-3
    th=0.0; om=0.0  # inertial LOS angle/rate
    i_act=0.0; integ=0; tf=0; omp=0; dh=0; tapp=0; lu=0; ly=0; k=0; i_cmd=0
    random.seed(1); errs=[]
    f=[2*(20**(n/7)) for n in range(8)]; phs=[2*math.pi*((n+1)*0.618%1) for n in range(8)]
    for step in range(int(T*fs)):
        t=step*dt
        # gyro
        yg+=ag*(om-yg); m=yg+math.radians(0.15)/60*math.sqrt(fs)*random.gauss(0,1)
        buf=[m]+buf[:-1]; g=buf[nd]
        # controller (el axis), LOS hold cmd 0 -> outer loop on true-ish angle
        if k>0:
            draw=J*(g-omp)*fs-tapp; dh+=a_d*(draw-dh)
        omp=g
        wcmd=2*math.pi*3*(0-th)
        e=wcmd-g; al=wc*(e+wi*integ)
        if lead: all_=(b0*al+b1*lu-a1*ly)/a0
        else: all_=al
        lu=al; ly=all_
        tau=J*all_
        sat=abs(tau/kt)>=imax; grow=e*tau>0
        if not(sat and grow): integ+=e*dt
        if dob: tau-=dh
        tf+=a_l*(tau-tf); i_cmd=max(-imax,min(imax,tf/kt)); tapp=i_cmd*kt; k+=1
        for s in range(sub):
            tt=t+s*h
            wb=sum(math.radians(1.0)*math.sqrt(2/8)*math.sin(2*math.pi*fi*tt+p) for fi,p in zip(f,phs))+math.radians(0.5)*math.sin(2*math.pi*85*tt)+math.radians(3)*2*math.pi*0.27*math.cos(2*math.pi*0.27*tt)
            i_act+=h*2*math.pi*500*(i_cmd-i_act)
            wr=om-wb
            fr=(Fc+(Fs-Fc)*math.exp(-(wr/vs)**2))*math.tanh(wr/eps)+b*wr
            grav=1.513*9.81*0.0005*math.cos(th)
            om+=h*(kt*i_act-fr-grav)/J; th+=h*om
        if t>=3: errs.append(th)
    return math.sqrt(sum(x*x for x in errs)/len(errs))*1e6, max(abs(x) for x in errs)*1e6
for d in (0,1):
    for l in (0,1):
        r=run(d,l,6.0,35 if l else 25,20); print(f"dob={d} lead={l}: rms={r[0]:.1f} urad peak={r[1]:.1f}")
