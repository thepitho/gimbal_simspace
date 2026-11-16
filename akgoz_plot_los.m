function m = akgoz_plot_los(out, titleText)
% Plot LOS stabilization results and print metrics. Returns m.rms_az/el, m.peak_az/el [rad].
if nargin<2, titleText='AKGOZ GIMBAL: LOS stabilization'; end
P=akgoz_params();
e=out.get('los_err');  ed=squeeze(e.Data); if size(ed,1)==2 && size(ed,2)~=2, ed=ed.'; end
d=out.get('ctrl_diag'); dd=squeeze(d.Data); if size(dd,1)==10 && size(dd,2)~=10, dd=dd.'; end
tAz=out.get('azimuth_torque'); tEl=out.get('elevation_torque');
bY=out.get('base_yaw_w'); bP=out.get('base_pitch_w');
assert(all(isfinite(ed(:))),'Non-finite LOS error.');

% Steady-state window, time-weighted RMS (variable-step samples)
t=e.Time; win=t>=P.sim.rms_from;
rmsw=@(x) sqrt(trapz(t(win),x(win).^2)/(t(find(win,1,'last'))-t(find(win,1))));
m.rms_az=rmsw(ed(:,1)); m.rms_el=rmsw(ed(:,2));
m.peak_az=max(abs(ed(win,1))); m.peak_el=max(abs(ed(win,2)));

figure('Name',titleText);
tiledlayout(4,1,'TileSpacing','compact');
nexttile; plot(bY.Time,bY.Data*180/pi,bP.Time,bP.Data*180/pi); grid on;
ylabel('Base rate [deg/s]'); legend('yaw','pitch'); title(titleText);
nexttile; plot(t,ed*1e6); grid on; ylim([-1 1]*max(5*max(m.rms_az,m.rms_el)*1e6,50));
ylabel('LOS error [\murad]'); legend('az (cross-el)','el');
xline(P.sim.rms_from,'--','RMS window');
nexttile; plot(d.Time,dd(:,1:2),'--',d.Time,dd(:,3:4)); grid on;
ylabel('LOS rate [rad/s]'); legend('cmd az','cmd el','gyro az','gyro el');
nexttile; plot(tAz.Time,tAz.Data,tEl.Time,tEl.Data); hold on;
plot(d.Time,-dd(:,5:6),':'); grid on;
ylabel('Torque [N*m]'); xlabel('Time [s]');
legend('net az','net el','-DOB d_{hat} az','-DOB d_{hat} el');

fprintf('LOS error, t >= %g s:  RMS az %.1f urad, el %.1f urad | peak az %.1f, el %.1f urad\n', ...
    P.sim.rms_from,m.rms_az*1e6,m.rms_el*1e6,m.peak_az*1e6,m.peak_el*1e6);
fprintf('Placeholder parameters (see akgoz_params.m source tags); no INS error, no isolators.\n');
end
