function check_model
% Local MATLAB smoke test. Does not certify physical model fidelity.
here=fileparts(mfilename('fullpath')); addpath(here);
for mode={'kinematic','torque','stabilized'}
    mdl=['akgoz_gimbal_' mode{1}];
    if ~isfile(fullfile(here,[mdl '.slx'])), build_gimbal(mode{1}); end
    load_system(fullfile(here,[mdl '.slx']));
    out=sim(mdl,'StopTime','0.5');
    sig={};
    for name={'base_yaw','base_pitch','azimuth','elevation'}
        sig=[sig,{[name{1} '_q'],[name{1} '_w']}]; %#ok<AGROW>
    end
    if strcmp(mode{1},'stabilized')
        sig=[sig,{'los_err','gyro_meas','gyro_true','ctrl_diag','azimuth_torque','elevation_torque'}]; %#ok<AGROW>
    end
    for s=sig
        v=out.get(s{1});
        assert(~isempty(v.Data) && all(isfinite(v.Data(:))),'Bad signal %s in %s',s{1},mdl);
    end
    fprintf('PASS: %s compiled and simulated for 0.5 s.\n',mdl);
end
end
