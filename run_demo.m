function out=run_demo(mode)
% Build if necessary, simulate and plot.
% run_demo('stabilized') : default; 1 kHz LOS stabilization (rate PI + lead + DOB, LOS loop)
% run_demo('torque')     : relative joint PD (not stabilized)
% run_demo('kinematic')  : prescribed motion only
if nargin==0, mode='stabilized'; end
mode=validatestring(mode,{'torque','kinematic','stabilized'});
here=fileparts(mfilename('fullpath')); addpath(here);
mdl=['akgoz_gimbal_' mode];
if ~isfile(fullfile(here,[mdl '.slx']))
    build_gimbal(mode);
else
    load_system(fullfile(here,[mdl '.slx']));
end
open_system(mdl); out=sim(mdl);

if strcmp(mode,'stabilized')
    akgoz_plot_los(out);
    return
end
figure('Name','AKGOZ GIMBAL: relative joint measurements');
tiledlayout(2,1);
for k=1:2
    axisName={'azimuth','elevation'}; name=axisName{k};
    q=out.get([name '_q']); w=out.get([name '_w']);
    assert(all(isfinite(q.Data(:))) && all(isfinite(w.Data(:))),'Non-finite output.');
    nexttile; plot(q.Time,q.Data*180/pi,'LineWidth',1.3); grid on;
    ylabel([name ' [deg]']); xlabel('Time [s]');
end
fprintf('Simulation completed. Outputs are relative joint angles/rates, not IMU rates.\n');
end
