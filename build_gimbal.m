function mdl = build_gimbal(mode)
% BUILD_GIMBAL Create the AKGOZ GIMBAL 2-axis (az/el, no roll) Simscape Multibody model.
% build_gimbal('stabilized'): LOS stabilization, 1 kHz discrete controller:
%     outer LOS position loop (master) -> inner head-gyro rate loop (slave) PI + lead,
%     disturbance observer, STIM210 gyro model, 16-bit encoders, FOC current loop,
%     Stribeck friction, cogging, cable torque, base maneuver + vibration.
% build_gimbal('torque')    : relative joint PD demo (not stabilized)
% build_gimbal('kinematic') : prescribed motion for animation only
% Parameters: akgoz_params.m. Controller: plain Simulink blocks (akgoz_build_controller.m).
% Sensor/estimator/actuator code: akgoz_*.m, copied into MATLAB Function blocks at build time.
% Requires MATLAB, Simulink, Simscape and Simscape Multibody. Target: R2025b.
if nargin==0, mode='stabilized'; end
mode=validatestring(mode,{'torque','kinematic','stabilized'});
assert(exist('smimport','file')~=0,'Simscape Multibody is required.');
here=fileparts(mfilename('fullpath'));
mdl=['akgoz_gimbal_' mode];
if isfile(fullfile(here,[mdl '.slx']))
    error('Model %s.slx already exists. Save your changes, then rename/remove it before rebuilding.',mdl);
end
if bdIsLoaded(mdl)
    error(['Model %s is still open in MATLAB memory (file not on disk).\n' ...
        'To discard it and rebuild:  close_system(''%s'',0); run_demo(''%s'')'], ...
        mdl,mdl,mode);
end
old=pwd; cleanup=onCleanup(@() cd(old)); cd(here); addpath(here);
P=akgoz_params();
stab=strcmp(mode,'stabilized');
load_system('simulink'); load_system('nesl_utility');
smimport(fullfile(here,'akgoz_gimbal.urdf'),'ModelName',mdl);
if stab
    % Continuous plant, 1 kHz discrete sensors/controller. Small max step for vibration.
    set_param(mdl,'SolverType','Variable-step','Solver','ode15s', ...
        'MaxStep','5e-4','RelTol','1e-5','StopTime',num2str(P.sim.stop), ...
        'ReturnWorkspaceOutputs','on');
else
    set_param(mdl,'SolverType','Variable-step','Solver','ode15s', ...
        'MaxStep','0.002','RelTol','1e-5','StopTime','12', ...
        'ReturnWorkspaceOutputs','on');
end
names={'base_yaw','base_pitch','azimuth','elevation'};
amp=[4 3 30 15]*pi/180; freq=[0.20 0.27 0.12 0.18];
nBase=2; % base disturbance joints (yaw, pitch); gimbal has no roll axis
for k=1:numel(names)
    found=find_system(mdl,'LookUnderMasks','all','FollowLinks','on','Name',names{k});
    assert(numel(found)==1,'Expected one imported joint named %s.',names{k});
    j=found{1}; parent=get_param(j,'Parent');
    assert(strcmp(parent,mdl),'Imported joint must be at top level: %s',j);
    y=100+170*k;
    prescribed=k<=nBase || strcmp(mode,'kinematic');
    if prescribed
        param(j,{'RzTorqueActuationMode','TorqueActuationMode'},'ComputedTorque');
        param(j,{'RzMotionActuationMode','MotionActuationMode'},'InputMotion');
    else
        param(j,{'RzMotionActuationMode','MotionActuationMode'},'ComputedMotion');
        param(j,{'RzTorqueActuationMode','TorqueActuationMode'},'InputTorque');
    end
    ref='';
    if ~stab   % stabilized: base from akgoz_base_motion, az/el from the controller
        ref=block('simulink/Sources/Sine Wave',[names{k} '_reference'],[30 y 100 y+35]);
        set_param(ref,'Amplitude',num2str(amp(k),16),'Frequency',num2str(2*pi*freq(k),16),'SampleTime','0');
    end
    ps=block('nesl_utility/Simulink-PS Converter',[names{k} '_input'],[490 y 555 y+35]);
    if prescribed
        tcf='0.01'; if stab, tcf='5e-4'; end   % vibration needs a fast input filter
        set_param(ps,'Unit','rad','FilteringAndDerivatives','filter', ...
            'SimscapeFilterOrder','2','InputFilterTimeConstant',tcf);
        if ~stab, wire(ref,'Outport',1,ps,'Inport',1); end
    else
        set_param(ps,'Unit','N*m');
    end
    % At this point frames are already wired by smimport. Only input is free.
    connectPhysical(ps,j);
    param(j,{'RzPositionSensing','PositionSensing','RzSensePosition','SensePosition'},'on');
    q=block('nesl_utility/PS-Simulink Converter',[names{k} '_q_rad'],[740 y 810 y+30]);
    set_param(q,'Unit','rad'); connectPhysical(j,q);
    param(j,{'RzVelocitySensing','VelocitySensing','RzSenseVelocity','SenseVelocity'},'on');
    w=block('nesl_utility/PS-Simulink Converter',[names{k} '_w_rad_s'],[740 y+50 810 y+80]);
    set_param(w,'Unit','rad/s'); connectPhysical(j,w);
    logSignal(q,[names{k} '_q'],[880 y 995 y+30]);
    logSignal(w,[names{k} '_w'],[880 y+50 995 y+80]);
    if ~prescribed && strcmp(mode,'torque')
        err=block('simulink/Math Operations/Sum',[names{k} '_error'],[150 y 175 y+40]);
        set_param(err,'Inputs','+-');
        kp=block('simulink/Math Operations/Gain',[names{k} '_Kp'],[210 y 270 y+35]);
        kd=block('simulink/Math Operations/Gain',[names{k} '_Kd'],[210 y+65 270 y+100]);
        set_param(kp,'Gain','4'); set_param(kd,'Gain','0.5');
        sumT=block('simulink/Math Operations/Sum',[names{k} '_PD'],[315 y 340 y+40]);
        set_param(sumT,'Inputs','+-');
        sat=block('simulink/Discontinuities/Saturation',[names{k} '_torque_Nm'],[380 y 450 y+35]);
        set_param(sat,'UpperLimit','0.4','LowerLimit','-0.4');
        wire(ref,'Outport',1,err,'Inport',1); wire(q,'Outport',1,err,'Inport',2);
        wire(err,'Outport',1,kp,'Inport',1); wire(w,'Outport',1,kd,'Inport',1);
        wire(kp,'Outport',1,sumT,'Inport',1); wire(kd,'Outport',1,sumT,'Inport',2);
        wire(sumT,'Outport',1,sat,'Inport',1); wire(sat,'Outport',1,ps,'Inport',1);
        logSignal(sat,[names{k} '_torque'],[490 y+70 610 y+100]);
    end
end
if stab
    buildLos();
    groupLos();
    modeText=['LOS STABILIZATION @ %g Hz: outer LOS position loop (master) -> inner head-gyro\n' ...
        'rate loop (slave) PI + lead, DOB, STIM210 gyro model, FOC current loop, friction.\n' ...
        'Switches: DOB_on, LEAD_on constants. Parameters: akgoz_params.m (source tags).'];
    modeText=sprintf(modeText,P.fs);
else
    modeText=sprintf(['q and w are RELATIVE joint measurements, not an inertial gyro.\n' ...
        'Default PD tracks relative joint angles; it is NOT LOS stabilization.']);
end
note=Simulink.Annotation(mdl, [sprintf(['AKGOZ GIMBAL | 2 eksen: azimuth + elevation (roll yok)\n' ...
    'EXAMPLE / PLACEHOLDER PARAMETERS | %s mode\n' ...
    'World: X forward, Y left, Z up. Camera optical direction: +X.\n'],mode) modeText]);
tidy(mdl);
note.Position=[30 -150 900 -20];
note.FontSize=12;
save_system(mdl,fullfile(here,[mdl '.slx']));
try
    set_param(mdl,'SimulationCommand','update');
catch ME
    fprintf(2,'Model saved but compile failed. See diagnostic below.\n');
    rethrow(ME);
end
open_system(mdl);
fprintf('Created %s.slx. Press Run; view animation in Multibody Explorer / Mechanics Explorer.\n',mdl);

    function buildLos()
        Ts=num2str(P.Ts,16);
        x0=1100; y0=260;
        % ---- joint vectors ----
        qm=block('simulink/Signal Routing/Mux','q_vector',[x0 y0 x0+5 y0+120]);
        wm=block('simulink/Signal Routing/Mux','w_vector',[x0 y0+160 x0+5 y0+280]);
        set_param(qm,'Inputs','4'); set_param(wm,'Inputs','4');
        for n=1:4
            wire([mdl '/' names{n} '_q_rad'],'Outport',1,qm,'Inport',n);
            wire([mdl '/' names{n} '_w_rad_s'],'Outport',1,wm,'Inport',n);
        end
        % ---- base (vehicle) motion: maneuver + vibration ----
        clk=block('simulink/Sources/Clock','clock',[30 40 60 70]);
        bm=fcnBlock('base_motion','akgoz_base_motion.m',[120 30 260 80],'');
        wire(clk,'Outport',1,bm,'Inport',1);
        bd=block('simulink/Signal Routing/Demux','base_demux',[300 30 305 80]);
        set_param(bd,'Outputs','2');
        wire(bm,'Outport',1,bd,'Inport',1);
        wire(bd,'Outport',1,[mdl '/base_yaw_input'],'Inport',1);
        wire(bd,'Outport',2,[mdl '/base_pitch_input'],'Inport',1);
        % ---- LOS command (inertial) ----
        elc=block('simulink/Sources/Step','LOS_el_cmd',[x0-80 y0+320 x0-40 y0+350]);
        azc=block('simulink/Sources/Step','LOS_az_cmd',[x0-80 y0+380 x0-40 y0+410]);
        set_param(elc,'Time',num2str(P.cmd.t(1)),'Before','0','After',num2str(P.cmd.el,16),'SampleTime',Ts);
        set_param(azc,'Time',num2str(P.cmd.t(2)),'Before','0','After',num2str(P.cmd.az,16),'SampleTime',Ts);
        cm=block('simulink/Signal Routing/Mux','LOS_cmd',[x0 y0+320 x0+5 y0+410]);
        set_param(cm,'Inputs','2');
        wire(azc,'Outport',1,cm,'Inport',1); wire(elc,'Outport',1,cm,'Inport',2);
        % ---- truth: head inertial rate and LOS error (continuous) ----
        tr=fcnBlock('truth_LOS','',[x0+80 y0 x0+230 y0+120],'');
        setScript(tr,strjoin({
            'function [wc, err] = truth_LOS(q, w, cmd)'
            '% True head inertial rate (camera frame) and true LOS error [az; el].'
            '[wc, err] = akgoz_los_kinematics(q, w, cmd);'
            'end'},newline));
        wire(qm,'Outport',1,tr,'Inport',1); wire(wm,'Outport',1,tr,'Inport',2);
        wire(cm,'Outport',1,tr,'Inport',3);
        logSignal(tr,'gyro_true',[x0+280 y0-60 x0+390 y0-35],1);
        logSignal(tr,'los_err',[x0+280 y0+560 x0+390 y0+585],2);
        % ---- STIM210 gyro @ fs ----
        nw=block('simulink/Sources/Random Number','gyro_white_noise',[x0+80 y0+150 x0+120 y0+180]);
        nb=block('simulink/Sources/Random Number','gyro_bias_noise',[x0+80 y0+200 x0+120 y0+230]);
        set_param(nw,'Mean','[0 0 0]','Variance','[1 1 1]','Seed',mat2str(P.gyro.seed_white),'SampleTime',Ts);
        set_param(nb,'Mean','[0 0 0]','Variance','[1 1 1]','Seed',mat2str(P.gyro.seed_bias),'SampleTime',Ts);
        gy=fcnBlock('gyro_STIM210','akgoz_gyro.m',[x0+280 y0+60 x0+420 y0+240],Ts);
        wire(tr,'Outport',1,gy,'Inport',1); wire(nw,'Outport',1,gy,'Inport',2);
        wire(nb,'Outport',1,gy,'Inport',3);
        logSignal(gy,'gyro_meas',[x0+460 y0+20 x0+570 y0+45],1);
        % ---- switches ----
        dob=block('simulink/Sources/Constant','DOB_on',[x0+300 y0+420 x0+340 y0+440]);
        led=block('simulink/Sources/Constant','LEAD_on',[x0+300 y0+460 x0+340 y0+480]);
        set_param(dob,'Value','1'); set_param(led,'Value','1');
        swm=block('simulink/Signal Routing/Mux','ctrl_switches',[x0+380 y0+420 x0+385 y0+480]);
        set_param(swm,'Inputs','2');
        wire(dob,'Outport',1,swm,'Inport',1); wire(led,'Outport',1,swm,'Inport',2);
        % ---- LOS estimate (ideal INS x 16-bit encoders) @ fs ----
        es=fcnBlock('LOS_estimator','akgoz_los_estimator.m',[x0+280 y0+300 x0+420 y0+400],Ts);
        wire(qm,'Outport',1,es,'Inport',1); wire(cm,'Outport',1,es,'Inport',2);
        % ---- LOS controller @ fs: plain Simulink blocks (akgoz_build_controller.m) ----
        ct=block('simulink/Ports & Subsystems/Subsystem','LOS_Controller_1kHz',[x0+500 y0+100 x0+680 y0+340]);
        akgoz_build_controller(ct,P);
        tidy(ct);
        wire(gy,'Outport',1,ct,'Inport',1); wire(es,'Outport',1,ct,'Inport',2);
        wire(es,'Outport',2,ct,'Inport',3); wire(swm,'Outport',1,ct,'Inport',4);
        % ---- FOC current loop (1st order) -> actuator torque ----
        den=sprintf('[%.10g 1]',1/(2*pi*P.mot.cl_bw));
        ax={'azimuth','elevation'};
        im=block('simulink/Signal Routing/Mux','i_motor',[x0+900 y0+120 x0+905 y0+320]);
        set_param(im,'Inputs','2');
        for n=1:2
            yy=y0+120+100*(n-1);
            cl=block('simulink/Continuous/Transfer Fcn',[ax{n} '_current_loop'],[x0+740 yy x0+840 yy+35]);
            set_param(cl,'Numerator','1','Denominator',den);
            wire(ct,'Outport',n,cl,'Inport',1);
            wire(cl,'Outport',1,im,'Inport',n);
        end
        ac=fcnBlock('actuator_friction','akgoz_actuator.m',[x0+960 y0+100 x0+1110 y0+340],'');
        wire(im,'Outport',1,ac,'Inport',1); wire(qm,'Outport',1,ac,'Inport',2);
        wire(wm,'Outport',1,ac,'Inport',3);
        for n=1:2
            wire(ac,'Outport',n,[mdl '/' ax{n} '_input'],'Inport',1);
            logSignal(ac,[ax{n} '_torque'],[x0+1160 y0+100+120*(n-1) x0+1270 y0+125+120*(n-1)],n);
        end
    end
    function groupLos()
        % Name the main signals (subsystem ports inherit them), then wrap each
        % function group into a subsystem. DOB_on / LEAD_on stay on the top level.
        nameOut('q_vector',1,'q'); nameOut('w_vector',1,'w');
        nameOut('base_demux',1,'base_yaw'); nameOut('base_demux',2,'base_pitch');
        nameOut('LOS_cmd',1,'los_cmd'); nameOut('ctrl_switches',1,'sw');
        nameOut('truth_LOS',1,'w_head_true'); nameOut('truth_LOS',2,'los_err');
        nameOut('gyro_STIM210',1,'gyro');
        nameOut('LOS_estimator',1,'los_err_est'); nameOut('LOS_estimator',2,'el_enc');
        nameOut('LOS_Controller_1kHz',1,'i_cmd_az'); nameOut('LOS_Controller_1kHz',2,'i_cmd_el');
        nameOut('azimuth_current_loop',1,'i_az'); nameOut('elevation_current_loop',1,'i_el');
        nameOut('actuator_friction',1,'tau_az'); nameOut('actuator_friction',2,'tau_el');
        groups={
            'Base_Motion',{'clock','base_motion','base_demux'}
            'LOS_Command',{'LOS_el_cmd','LOS_az_cmd','LOS_cmd'}
            'Sensors',{'truth_LOS','gyro_white_noise','gyro_bias_noise','gyro_STIM210', ...
                'LOS_estimator','log_gyro_true','log_los_err','log_gyro_meas'}
            'Actuators',{'azimuth_current_loop','elevation_current_loop','i_motor', ...
                'actuator_friction','log_azimuth_torque','log_elevation_torque'}};
        keep=[{'DOB_on','LEAD_on','ctrl_switches','LOS_Controller_1kHz'},groups{:,2}];
        top=find_system(mdl,'SearchDepth',1,'Type','Block');
        top=setdiff(top,{mdl});
        plant=top(~ismember(strrep(top,[mdl '/'],''),keep));   % mechanism + joint I/O + q/w muxes
        try
            makeSub('Plant_Simscape',plant);
        catch ME   % physical (Simscape) grouping is version dependent; keep the plant on top
            warning('build_gimbal:plantGroup', ...
                'Plant not grouped into a subsystem (%s). It stays on the top level.',ME.message);
        end
        for g=1:size(groups,1)
            makeSub(groups{g,1},strcat([mdl '/'],groups{g,2}));
        end
    end
    function makeSub(name,paths)
        h=cellfun(@(p)get_param(p,'Handle'),paths);
        before=find_system(mdl,'SearchDepth',1,'BlockType','SubSystem');
        Simulink.BlockDiagram.createSubsystem(h);
        after=find_system(mdl,'SearchDepth',1,'BlockType','SubSystem');
        new=setdiff(after,before);
        assert(numel(new)==1,'createSubsystem for %s: expected one new subsystem, found %d.',name,numel(new));
        set_param(new{1},'Name',name);
        tidy([mdl '/' name]);
    end
    function nameOut(blk,port,name)
        ph=get_param([mdl '/' blk],'PortHandles');
        ln=get_param(ph.Outport(port),'Line');
        assert(ln~=-1,'No line on %s outport %d.',blk,port);
        set_param(ln,'Name',name);
    end
    function b=fcnBlock(name,file,pos,ts)
        % MATLAB Function block; code from FILE (if given); ts='' -> inherited.
        b=block('simulink/User-Defined Functions/MATLAB Function',name,pos);
        if ~isempty(file), setScript(b,fileread(fullfile(here,file))); end
        if ~isempty(ts)
            cfg=get_param(b,'MATLABFunctionConfiguration');
            cfg.UpdateMethod='Discrete';
            cfg.SampleTime=ts;
        end
    end
    function setScript(b,code)
        cfg=get_param(b,'MATLABFunctionConfiguration');
        cfg.FunctionScript=code;
    end
    function b=block(src,name,pos)
        b=[mdl '/' name]; add_block(src,b,'Position',pos);
    end
    function wire(a,at,ai,b,bt,bi)
        pa=get_param(a,'PortHandles'); pb=get_param(b,'PortHandles');
        add_line(mdl,pa.(at)(ai),pb.(bt)(bi),'autorouting','on');
    end
    function connectPhysical(a,b)
        % Connect only unoccupied physical ports: no fixed frame/PS index guesses.
        ha=freePhysical(a); hb=freePhysical(b);
        assert(numel(ha)==1 && numel(hb)==1, ...
            'Expected one free physical port on %s and %s; found %d and %d.',a,b,numel(ha),numel(hb));
        add_line(mdl,ha,hb,'autorouting','on');
    end
    function h=freePhysical(b)
        p=get_param(b,'PortHandles'); h=[p.LConn(:);p.RConn(:)];
        h=h(arrayfun(@(v)get_param(v,'Line')==-1,h));
    end
    function logSignal(src,name,pos,port)
        if nargin<4, port=1; end
        b=block('simulink/Sinks/To Workspace',['log_' name],pos);
        set_param(b,'VariableName',name,'SaveFormat','Timeseries');
        wire(src,'Outport',port,b,'Inport',1);
    end
end

function tidy(sys)
% Automatic block layout (Simulink "Arrange"). Layout only: a failure is reported, not fatal.
try
    Simulink.BlockDiagram.arrangeSystem(sys);
catch ME
    warning('build_gimbal:arrange','Auto-arrange failed for %s: %s',sys,ME.message);
end
end

function param(b,candidates,value)
% Resolve known mask-name variants; fail explicitly, never skip configuration.
p=get_param(b,'DialogParameters');
for n=1:numel(candidates)
    if isfield(p,candidates{n})
        set_param(b,candidates{n},value); return
    end
end
error('Unsupported block mask on %s. Need %s. Available: %s', ...
    b,strjoin(candidates,' / '),strjoin(fieldnames(p),', '));
end
