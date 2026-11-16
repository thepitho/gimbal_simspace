# Elevation-axis rate-loop margins (discrete 1 kHz: PI, lead, 150 Hz LPF, gyro 262 Hz + 2 samples, ZOH, 500 Hz current loop).
# Supports the rate_wc_* choices in akgoz_params.m. Plain Python 3, no packages.
import cmath, math
fs=1000; T=1/fs
def L(w, lead, norm, wc=2*math.pi*25):
    s=1j*w; z=cmath.exp(s*T)
    wi=wc/4
    C=wc*(1+wi*T/(z-1)*1)  # forward-Euler integ approx: integ += e*dt -> T/(z-1)
    ar=3.0; wz=wc/math.sqrt(ar); wp=wc*math.sqrt(ar)
    kl={'wc':1/math.sqrt(ar),'dc':1.0}[norm]
    A=2*fs; b0=kl*(A/wz+1); b1=kl*(1-A/wz); a0=A/wp+1; a1=1-A/wp
    Ld=(b0+b1/z)/(a0+a1/z) if lead else 1
    al=1-math.exp(-2*math.pi*150/fs); LPF=al/(1-(1-al)/z)
    ag=1-math.exp(-2*math.pi*262/fs); G=ag/(1-(1-ag)/z)*z**-2   # gyro lag+2 sample delay (sampled)
    ZOH=(1-1/z)/(s*T)
    CL=1/(1+s/(2*math.pi*500))
    P=1/s  # J cancels
    return C*Ld*LPF*G*ZOH*CL*P
for lead,norm in ((0,'wc'),(1,'wc'),(1,'dc')):
    ws=[2*math.pi*10**(x/400) for x in range(0,1100)]
    prev=None; out=''
    for w in ws:
        m=abs(L(w,lead,norm))
        if prev and prev>=1>m:
            pm=180+math.degrees(cmath.phase(L(w,lead,norm))); out+=f'fc={w/2/math.pi:.1f}Hz PM={pm:.1f}deg '
        prev=m
    # gain margin
    prevp=None
    for w in ws:
        p=math.degrees(cmath.phase(L(w,lead,norm)))
        if prevp is not None and prevp>-180+360*0 and p<-170 and abs(p)>170 and prevp*p<0 or (prevp is not None and prevp>-180>=p):
            out+=f'GM={-20*math.log10(abs(L(w,lead,norm))):.1f}dB@{w/2/math.pi:.0f}Hz'; break
        prevp=p
    print(lead,norm,out)
