function akgoz_build_controller(sys, P)
% AKGOZ_BUILD_CONTROLLER Fill subsystem SYS with the 1 kHz LOS controller in plain
% Simulink blocks (no MATLAB Function). Called by build_gimbal.m.
%
% Inports : 1 gyro [3] head rate, camera frame   2 los_err_est [2] [az; el]
%           3 el_enc [1] encoder elevation          4 sw [2] = [DOB_on; LEAD_on]
% Outports: 1 i_cmd_az  2 i_cmd_el  3 ctrl_diag [10] =
%           [w_cmd(2); w_meas(2); d_hat(2); err(2); tau_ff(2)]
% Per axis (az row, el row):
%   err -> Kpos -> slew limit -> w_cmd;  e = w_cmd - w_meas
%   alpha = wc*e + (wc^2/4)*int(e)        (PI, wc = 25 Hz lead OFF / 35 Hz lead ON)
%   alpha_l = Lead(z)*alpha  or alpha     (LEAD_on)
%   tau_ff = J*alpha_l (az: J_az(el)*sec(el))
%   d_hat = Q(z)[J*diff(omega)*fs - tau_applied(k-1)],  tau = tau_ff - d_hat (DOB_on)
%   i = sat(LPF(tau)/Kt, imax);  tau_applied = i*Kt
% Anti-windup: integrator state clamped at the torque limit (J_axis at el = 0).
% Slew limit is per axis (the old MATLAB version capped the vector norm).
delete_line(sys,'In1/1','Out1/1');
delete_block([sys '/In1']); delete_block([sys '/Out1']);
Ts = num2str(P.Ts,16);
fs = P.fs; J = P.J; kt = P.mot.kt; imax = P.mot.imax;
wcp = P.ctl.rate_wc_pi; wcl = P.ctl.rate_wc_lead;
ph = P.ctl.lead_phase*pi/180; ar = (1+sin(ph))/(1-sin(ph));
wz = wcl/sqrt(ar); wp = wcl*sqrt(ar); kl = 1/sqrt(ar); A = 2*fs;
leadNum = [kl*(A/wz+1), kl*(1-A/wz)]; leadDen = [A/wp+1, 1-A/wp];   % Tustin
a_d = 1 - exp(-2*pi*P.ctl.dob_cutoff/fs);
a_l = 1 - exp(-2*pi*P.ctl.rate_lpf/fs);
Jv0 = [J.yoke_zz + J.head_zz; J.el];
xmax = imax.*kt./Jv0/(wcp^2/4);                                     % integrator clamp
ax = {'az','el'};

% ---- ports ----
inG  = add('simulink/Sources/In1','gyro',[20 100 50 114]);
inE  = add('simulink/Sources/In1','los_err_est',[20 400 50 414]);
inEl = add('simulink/Sources/In1','el_enc',[20 600 50 614]);
inSw = add('simulink/Sources/In1','sw',[20 800 50 814]);
oAz  = add('simulink/Sinks/Out1','i_cmd_az',[1900 300 1930 314]);
oEl  = add('simulink/Sinks/Out1','i_cmd_el',[1900 700 1930 714]);
oDg  = add('simulink/Sinks/Out1','ctrl_diag',[1900 950 1930 964]);

% ---- shared signals ----
gd = add('simulink/Signal Routing/Demux','gyro_xyz',[100 80 105 140]); set_param(gd,'Outputs','3');
ed = add('simulink/Signal Routing/Demux','err_azel',[100 390 105 430]); set_param(ed,'Outputs','2');
sd = add('simulink/Signal Routing/Demux','sw_dob_lead',[100 790 105 830]); set_param(sd,'Outputs','2');
w(inG,1,gd,1); w(inE,1,ed,1); w(inSw,1,sd,1);
sinEl = add('simulink/Math Operations/Trigonometric Function','sin_el',[120 560 160 590]);
cosEl = add('simulink/Math Operations/Trigonometric Function','cos_el',[120 630 160 660]);
set_param(sinEl,'Operator','sin'); set_param(cosEl,'Operator','cos');
w(inEl,1,sinEl,1); w(inEl,1,cosEl,1);
% J_az(el) = yoke_zz + head_xx*s^2 + head_zz*c^2 + 2*head_xz*s*c
s2 = add('simulink/Math Operations/Product','sin_sq',[220 520 250 550]);
c2 = add('simulink/Math Operations/Product','cos_sq',[220 570 250 600]);
sc = add('simulink/Math Operations/Product','sin_cos',[220 620 250 650]);
w(sinEl,1,s2,1); w(sinEl,1,s2,2); w(cosEl,1,c2,1); w(cosEl,1,c2,2);
w(sinEl,1,sc,1); w(cosEl,1,sc,2);
gxx = gain('J_head_xx',J.head_xx,[280 520 330 550]); w(s2,1,gxx,1);
gzz = gain('J_head_zz',J.head_zz,[280 570 330 600]); w(c2,1,gzz,1);
gxz = gain('J_head_2xz',2*J.head_xz,[280 620 330 650]); w(sc,1,gxz,1);
cy = add('simulink/Sources/Constant','J_yoke_zz',[280 670 330 690]);
set_param(cy,'Value',num2str(J.yoke_zz,16));
jaz = add('simulink/Math Operations/Sum','J_az',[370 560 395 680]); set_param(jaz,'Inputs','++++');
w(gxx,1,jaz,1); w(gzz,1,jaz,2); w(gxz,1,jaz,3); w(cy,1,jaz,4);
% sec(el) capped: sign(cos)/max(|cos|, 1/cap)
ac = add('simulink/Math Operations/Abs','abs_cos',[220 700 250 730]);
cs = add('simulink/Discontinuities/Saturation','cos_floor',[280 700 330 730]);
set_param(cs,'UpperLimit','1','LowerLimit',num2str(1/P.ctl.secant_cap,16));
rc = add('simulink/Math Operations/Math Function','reciprocal',[350 700 400 730]);
set_param(rc,'Operator','reciprocal');
sg = add('simulink/Math Operations/Sign','sign_cos',[280 750 310 780]);
sec = add('simulink/Math Operations/Product','sec_el',[430 700 460 760]);
w(cosEl,1,ac,1); w(ac,1,cs,1); w(cs,1,rc,1); w(cosEl,1,sg,1);
w(rc,1,sec,1); w(sg,1,sec,2);
% measured LOS rates and axis inertial rates
ng = gain('neg_gyro_y',-1,[160 100 190 120]); w(gd,2,ng,1);            % el rate = -g_y
gs = add('simulink/Math Operations/Product','gx_sin_el',[220 40 250 70]);
gc = add('simulink/Math Operations/Product','gz_cos_el',[220 150 250 180]);
w(gd,1,gs,1); w(sinEl,1,gs,2); w(gd,3,gc,1); w(cosEl,1,gc,2);
omAz = add('simulink/Math Operations/Sum','omega_az_axis',[290 90 315 140]);
w(gs,1,omAz,1); w(gc,1,omAz,2);
% crossover select: wc = LEAD_on ? wc_lead : wc_pi
cl = add('simulink/Sources/Constant','wc_lead',[140 860 200 880]); set_param(cl,'Value',num2str(wcl,16));
cp = add('simulink/Sources/Constant','wc_pi',[140 920 200 940]);  set_param(cp,'Value',num2str(wcp,16));
wsel = sw3('wc_select',[240 850 270 950],cl,sd,2,cp);
wsq = add('simulink/Math Operations/Math Function','wc_squared',[310 885 360 915]);
set_param(wsq,'Operator','square'); w(wsel,1,wsq,1);
wiq = gain('quarter',0.25,[390 885 430 915]); w(wsq,1,wiq,1);
zero = add('simulink/Sources/Constant','zero',[140 980 170 1000]); set_param(zero,'Value','0');

% ---- per axis ----
diagSig = cell(5,2);   % rows: w_cmd, w_meas, d_hat, err, tau_ff
for k = 1:2
    y = 250 + 400*(k-1); a = ax{k};
    if k == 1, wm = {gd,3}; om = {omAz,1}; else, wm = {ng,1}; om = {ng,1}; end
    kp = gain([a '_Kpos'],2*pi*P.ctl.pos_bw,[500 y 550 y+30]); w(ed,k,kp,1);
    sl = add('simulink/Discontinuities/Saturation',[a '_slew_limit'],[580 y 630 y+30]);
    set_param(sl,'UpperLimit',num2str(P.ctl.slew_max,16),'LowerLimit',num2str(-P.ctl.slew_max,16));
    w(kp,1,sl,1);
    re = add('simulink/Math Operations/Sum',[a '_rate_err'],[670 y 695 y+40]); set_param(re,'Inputs','+-');
    w(sl,1,re,1); w(wm{1},wm{2},re,2);
    pP = add('simulink/Math Operations/Product',[a '_P'],[740 y-40 770 y-10]);
    w(wsel,1,pP,1); w(re,1,pP,2);
    it = add('simulink/Discrete/Discrete-Time Integrator',[a '_int_e'],[740 y+30 790 y+60]);
    set_param(it,'IntegratorMethod','Integration: Forward Euler','SampleTime',Ts,'LimitOutput','on', ...
        'UpperSaturationLimit',num2str(xmax(k),16),'LowerSaturationLimit',num2str(-xmax(k),16));
    w(re,1,it,1);
    pI = add('simulink/Math Operations/Product',[a '_I'],[820 y+30 850 y+60]);
    w(wiq,1,pI,1); w(it,1,pI,2);
    pi_ = add('simulink/Math Operations/Sum',[a '_PI'],[880 y-20 905 y+50]);
    w(pP,1,pi_,1); w(pI,1,pi_,2);
    ld = add('simulink/Discrete/Discrete Transfer Fcn',[a '_lead'],[940 y-50 1020 y-10]);
    set_param(ld,'Numerator',mat2str(leadNum,16),'Denominator',mat2str(leadDen,16),'SampleTime',Ts);
    w(pi_,1,ld,1);
    lsw = sw3([a '_lead_on'],[1060 y-40 1090 y+40],ld,sd,2,pi_);
    if k == 1
        tf = add('simulink/Math Operations/Product','az_tau_ff',[1130 y-20 1160 y+50]);
        set_param(tf,'Inputs','3'); w(jaz,1,tf,1); w(sec,1,tf,2); w(lsw,1,tf,3);
    else
        tf = gain('el_tau_ff',J.el,[1130 y 1180 y+30]); w(lsw,1,tf,1);
    end
    % DOB
    df = add('simulink/Discrete/Difference',[a '_omega_diff'],[500 y+120 540 y+150]);
    w(om{1},om{2},df,1);
    fsG = gain([a '_x_fs'],fs,[570 y+120 620 y+150]); w(df,1,fsG,1);
    if k == 1
        jd = add('simulink/Math Operations/Product','az_J_domega',[650 y+120 680 y+150]);
        w(jaz,1,jd,1); w(fsG,1,jd,2);
    else
        jd = gain('el_J_domega',J.el,[650 y+120 700 y+150]); w(fsG,1,jd,1);
    end
    dr = add('simulink/Math Operations/Sum',[a '_dob_raw'],[740 y+120 765 y+160]); set_param(dr,'Inputs','+-');
    w(jd,1,dr,1);
    q = add('simulink/Discrete/Discrete Transfer Fcn',[a '_dob_Q'],[800 y+120 880 y+160]);
    set_param(q,'Numerator',mat2str([a_d 0],16),'Denominator',mat2str([1 -(1-a_d)],16),'SampleTime',Ts);
    w(dr,1,q,1);
    dsw = sw3([a '_dob_on'],[1130 y+90 1160 y+170],q,sd,1,zero);
    td = add('simulink/Math Operations/Sum',[a '_tau_minus_dob'],[1220 y 1245 y+40]); set_param(td,'Inputs','+-');
    w(tf,1,td,1); w(dsw,1,td,2);
    lp = add('simulink/Discrete/Discrete Transfer Fcn',[a '_tau_lpf'],[1290 y 1370 y+40]);
    set_param(lp,'Numerator',mat2str([a_l 0],16),'Denominator',mat2str([1 -(1-a_l)],16),'SampleTime',Ts);
    w(td,1,lp,1);
    ik = gain([a '_inv_kt'],1/kt(k),[1410 y 1460 y+30]); w(lp,1,ik,1);
    is = add('simulink/Discontinuities/Saturation',[a '_i_limit'],[1500 y 1550 y+30]);
    set_param(is,'UpperLimit',num2str(imax(k),16),'LowerLimit',num2str(-imax(k),16));
    w(ik,1,is,1);
    if k == 1, w(is,1,oAz,1); else, w(is,1,oEl,1); end
    kg = gain([a '_kt'],kt(k),[1500 y+120 1550 y+150]); w(is,1,kg,1);
    ud = add('simulink/Discrete/Unit Delay',[a '_tau_applied_z1'],[1580 y+120 1620 y+150]);
    set_param(ud,'SampleTime',Ts); w(kg,1,ud,1); w(ud,1,dr,2);
    diagSig(:,k) = {{sl,1};{wm{1},wm{2}};{q,1};{ed,k};{tf,1}};
end
dm = add('simulink/Signal Routing/Mux','diag_mux',[1800 900 1805 1050]); set_param(dm,'Inputs','10');
n = 0;
for r = 1:5
    for k = 1:2
        n = n + 1; s = diagSig{r,k}; w(s{1},s{2},dm,n);
    end
end
w(dm,1,oDg,1);
logTo(dm,'ctrl_diag',[1850 1000 1960 1025]);
logTo([sys '/az_i_limit'],'azimuth_i_cmd',[1600 330 1710 355]);
logTo([sys '/el_i_limit'],'elevation_i_cmd',[1600 730 1710 755]);

    function b = add(src,name,pos)
        b = [sys '/' name]; add_block(src,b,'Position',pos);
    end
    function b = gain(name,val,pos)
        b = add('simulink/Math Operations/Gain',name,pos);
        set_param(b,'Gain',num2str(val,16));
    end
    function b = sw3(name,pos,uTrue,ctl,ctlPort,uFalse)
        % Switch: passes uTrue when control input > 0.5, else uFalse
        b = add('simulink/Signal Routing/Switch',name,pos);
        set_param(b,'Criteria','u2 > Threshold','Threshold','0.5');
        w(uTrue,1,b,1); w(ctl,ctlPort,b,2); w(uFalse,1,b,3);
    end
    function w(a,ai,b,bi)
        pa = get_param(a,'PortHandles'); pb = get_param(b,'PortHandles');
        add_line(sys,pa.Outport(ai),pb.Inport(bi),'autorouting','on');
    end
    function logTo(src,name,pos)
        b = add('simulink/Sinks/To Workspace',['log_' name],pos);
        set_param(b,'VariableName',name,'SaveFormat','Timeseries');
        w(src,1,b,1);
    end
end
