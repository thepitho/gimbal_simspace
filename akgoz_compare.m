function T = akgoz_compare()
% Run the stabilized model with PI / PI+lead / PI+DOB / PI+lead+DOB and tabulate LOS error.
here=fileparts(mfilename('fullpath')); addpath(here);
mdl='akgoz_gimbal_stabilized';
if ~isfile(fullfile(here,[mdl '.slx'])), build_gimbal('stabilized'); end
load_system(fullfile(here,[mdl '.slx']));
cfg={'PI',0,0; 'PI + lead',0,1; 'PI + DOB',1,0; 'PI + lead + DOB',1,1};
n=size(cfg,1); rmsAz=zeros(n,1); rmsEl=rmsAz; pkAz=rmsAz; pkEl=rmsAz;
for k=1:n
    in=Simulink.SimulationInput(mdl);
    in=in.setBlockParameter([mdl '/DOB_on'],'Value',num2str(cfg{k,2}));
    in=in.setBlockParameter([mdl '/LEAD_on'],'Value',num2str(cfg{k,3}));
    out=sim(in);
    m=akgoz_plot_los(out,['AKGOZ LOS: ' cfg{k,1}]);
    rmsAz(k)=m.rms_az*1e6; rmsEl(k)=m.rms_el*1e6; pkAz(k)=m.peak_az*1e6; pkEl(k)=m.peak_el*1e6;
end
T=table(rmsAz,rmsEl,pkAz,pkEl,'RowNames',cfg(:,1), ...
    'VariableNames',{'RMS_az_urad','RMS_el_urad','peak_az_urad','peak_el_urad'});
disp(T);
end
