clear all
close all

%% Parameters
% Server
socket = tcpip('localhost', 50008);
set(socket,'OutputBufferSize', 1024);
fopen(socket);
scale_motion = 1E5;
scale_time = 1E3;

% Motion tracking parameters
mot_track_plot = 0;
mot_curr_roi = 0;
% z tracking
mot_track_on_z = 0;
mot_comp_on_z = 1;
mot_width_z = 10E-3;
mot_start_z = 70E-3; 
mot_length_z = 30E-3;
mot_vect_z = [];
mot_time_z = [];
mot_exp_z = 1;
mot_thresh_z = 0.25;
mot_max_dz = 2E-3;
mot_ref_pos_z = 80E-3;
% x tracking
mot_track_on_x = 0;
mot_comp_on_x = 1;
mot_width_x = 30E-3;
mot_top_x = 90E-3; 
mot_left_x = -mot_width_x/2; 
mot_height_x = 10E-3;
mot_vect_x = [];
mot_max_dx = 2E-3;
mot_ref_x = [];
mot_ref_x_update = 0;
mot_lag_x = 0.35;
mot_time_x = [];
mot_mc_vect_x = [];
mot_min_mc_x = 0.9;

% Display rate
F_display = 1; 
n_bmode_z_scale = 13;
n_bmode_x_scale = 11;

% Imaging
n_ray = 1;
t_frame = 25000;
start_depth = 0E-3;
end_depth = 130E-3;
ray_focus = 70; %mm
filename = 'p4_2Flash_track';
focus_pos = [-1.3,116]*1E-3;
focus_pos_bh = [-0.9,109]*1E-3;
focus_pos_bh_fliped = [1.2,109]*1E-3;
Nframe = 1;
c0 = 1500;
Nf_post = 4;
img_angle = pi/30;
bmode_scale = 0.26;
bmode_flip = 0;

% Gating
gating_bool = 0;
gating_delay = 0.8;
gating_thresh = 0.5E-3;
gating_prf = 1;
last_firing_clock = clock;
gating_num = 0;
save_ite = 0;
p_burst = 0;

% Cavitation monitoring
lstn_time = 10.2E-3; %ms

% PAM
pam_mode = 0;
elem_pitch = 0.32E-3;
x_pam = -10.08E-3:elem_pitch:10.08E-3;
z_pam = 20E-3:2E-3:end_depth;
pam_bb_avg = zeros(length(x_pam), length(z_pam));
pam_bb_frames = [];
pam_fast = 1;
freqs_bb = [round(linspace(2.1E6,2.9E6, 50)), round(linspace(3.1E6,3.9E6, 50))];
pam_win = 1E-3;
pam_windowed = 0;
pam_max_avg = [];
pam_x_avg = [];
pam_z_avg = [];
pam_alpha = 0.5;

% In vivo focus calculation
c_water = 1482;
d_water = 90E-3;
T_water = 20; %C
c_fat = 1420;
d_fat = 10E-3;
c_tissue = 1600;
d_tissue = 30E-3;
tx_foc = 0.12;
tx_dfoc = 0;
tx_aperture = 0.14;
shifted_focus_pos = focus_pos_bh-[0,10]*1E-3;

% Doppler
dop_numen = 15; % no. of ensembles
dop_freq = 2.5; % MHz
dop_M = 1; % order of high PRF Doppler
dop_N = 2; % doppler range gate start depth / doppler range gate size
dop_rangedepth = 64 *(c0/(dop_freq*1e6)); % [m] range gate size should be a multiple of 16
dop_rangegate = 9.0 + [0 dop_rangedepth*1e2]; % in [cm]
dop_rangewidth = 3.4;
dop_prf = round(dop_M *c0 / (2*dop_N*dop_rangedepth));
dop_V = 20;

%% Verasonics stuff
% Misc/debug
% bmode_xscale = 9;
% bmode_yscale = 13;
bmode_fontsize = 12;
just_gated = 0;

vsx_roi_gui

% Parallel pool check
% gcp;

% Imaging frequency stuff
img_freq = 2.5; %MHz
img_min_freq = 2; %MHz
img_max_freq = 3.4; %MHz
img_valid_tw_freq = [2, 2.05, 2.09, 2.14, 2.20, 2.25, 2.31, 2.37, 2.43, 2.5,...
                     2.57, 2.65, 2.73, 2.81, 2.9, 3, 3.1, 3.21, 3.33, 3.46,...
                     3.6, 3.75, 3.91, 4.09, 4.29, 4.5, 4.74, 5, 5.29, 5.63, 6];
img_valid_tw = [45, round(2*45/6), 2, 1; 
                44, round(2*44/6), 2, 1; 
                43, round(2*43/6), 2, 1; 
                42, round(2*42/6), 2, 1; 
                41, round(2*41/6), 2, 1; 
                40, round(2*40/6), 2, 1; 
                39, round(2*39/6), 2, 1; 
                38, round(2*38/6), 2, 1; 
                37, round(2*37/6), 2, 1; 
                36, round(2*36/6), 2, 1; 
                35, round(2*35/6), 2, 1; 
                34, round(2*34/6), 2, 1; 
                33, round(2*33/6), 2, 1; 
                32, round(2*32/6), 2, 1; 
                31, round(2*31/3), 2, 1; 
                30, round(2*30/3), 2, 1; 
                29, round(2*29/3), 2, 1; 
                28, round(2*28/3), 2, 1; 
                27, round(2*27/3), 2, 1; 
                26, round(2*26/3), 2, 1; 
                25, round(2*25/3), 2, 1; 
                24, round(2*24/3), 2, 1; 
                23, round(2*23/3), 2, 1; 
                22, round(2*22/3), 2, 1; 
                21, round(2*21/3), 2, 1; 
                20, round(2*20/3), 2, 1; 
                19, round(2*19/3), 2, 1; 
                18, round(2*18/3), 2, 1; 
                17, round(2*17/3), 2, 1; 
                16, round(2*16/3), 2, 1; 
                15, round(2*15/3), 2, 1]; 

% Specify system parameters.
Resource.Parameters.speedOfSound = c0;
Resource.Parameters.numTransmit = 128;  % number of transmit channels.
Resource.Parameters.numRcvChannels = 64;

% Specify Trans structure array.
Trans.name = 'P4-2';
Trans.frequency = 3;
Trans = computeTrans(Trans);
Trans.maxHighVoltage = 50;
lambda = Resource.Parameters.speedOfSound/Trans.frequency/1e6;

% Set up the listening parameters
max_end_depth = 1024; %limitation from verasonics
lstn_depth = 0.5 * lstn_time * c0 / lambda;
lstn_rcv = ceil(lstn_depth / max_end_depth);

% Set up SFormat structure array.
aperture = 64*Trans.spacing; % aperture based on 64 elements
SFormat(1).transducer = 'P4-2';
SFormat(1).scanFormat = 'VAPX';
SFormat(1).theta = -img_angle;
SFormat(1).radius = (aperture/2)/tan(-SFormat(1).theta); % dist. to virt. apex
SFormat(1).numRays = 1;      % no. of Rays (1 for Flash transmit)
SFormat(1).FirstRayLoc = [0,0,0];
SFormat(1).rayDelta = 2*(-SFormat.theta);
SFormat(1).startDepth = start_depth;
SFormat(1).endDepth = ceil(end_depth/lambda);   % Acquisition depth in wavelengths

% Set up PData structure.
PData(1).sFormat = 1;
PData(1).pdeltaX = 0.5;
PData(1).pdeltaZ = 0.5;
PData(1).Size(1) = 10+ceil((SFormat(1).endDepth-SFormat(1).startDepth)/PData(1).pdeltaZ);
PData(1).Size(2) = 10+ceil(2*(SFormat(1).endDepth + SFormat(1).radius)*sin(-SFormat(1).theta)/PData(1).pdeltaX);
PData(1).Size(3) = 1;
PData(1).Origin = [-(PData(1).Size(2)/2)*PData(1).pdeltaX+2,0,0];

PData(2).sFormat = 1;
PData(2).pdeltaX = 1;
PData(2).pdeltaZ = 1;
PData(2).Size(1) = PData(1).Size(1);
PData(2).Size(2) = PData(1).Size(2);
PData(2).Size(3) = 1;
PData(2).Origin = PData(1).Origin;

Resource.RcvBuffer(1).datatype = 'int16';
Resource.RcvBuffer(1).rowsPerFrame = 4*4096;
Resource.RcvBuffer(1).colsPerFrame = Resource.Parameters.numRcvChannels;
Resource.RcvBuffer(1).numFrames = Nframe; 
Resource.InterBuffer(1).datatype = 'complex';
Resource.InterBuffer(1).numFrames = 1;  
Resource.InterBuffer(1).rowsPerFrame = PData(1).Size(1);
Resource.InterBuffer(1).colsPerFrame = PData(1).Size(2);
Resource.ImageBuffer(1).datatype = 'double';
Resource.ImageBuffer(1).rowsPerFrame = PData(1).Size(1); % this is for maximum depth
Resource.ImageBuffer(1).colsPerFrame = PData(1).Size(2);
Resource.ImageBuffer(1).numFrames = Nframe;
Resource.DisplayWindow(1).Title = 'P4-2Ray';
Resource.DisplayWindow(1).pdelta = 1;
Resource.DisplayWindow(1).Position = [20,50,PData(1).Size(2),PData(1).Size(1)];
Resource.DisplayWindow(1).ReferencePt = [PData(1).Origin(1),PData(1).Origin(3)]; % 2D imaging is in the X,Z plane
Resource.DisplayWindow(1).Colormap = gray(256);

x_bmode = linspace(-0.5*PData(1).Size(2)*PData(1).pdeltaX*lambda, 0.5*PData(1).Size(2)*PData(1).pdeltaX*lambda, Resource.DisplayWindow(1).Position(3));
z_bmode = linspace(start_depth, PData(1).Size(1)*PData(1).pdeltaZ*lambda, Resource.DisplayWindow(1).Position(4));

bmode_display_size = [int16(ceil(PData(1).Size(2)*(PData(1).pdeltaX/bmode_scale))),...
                      ceil(PData(1).Size(1)*PData(1).pdeltaZ/bmode_scale)];
bmode_scale_z_pix = round(linspace(0, double(bmode_display_size(2)), n_bmode_z_scale)); 
bmode_scale_z_pix = bmode_scale_z_pix(2:end-1);

bmode_scale_x_pix = round(linspace(0, double(bmode_display_size(1)), n_bmode_x_scale));
bmode_scale_x_pix = bmode_scale_x_pix(2:end-1);

% Preallocate bmode post firing buffer
img_pre_gating = zeros(PData(1).Size(1), PData(1).Size(2));
img_post_gating = zeros(PData(1).Size(1), PData(1).Size(2), Nf_post);
iq_Dop_gating = zeros(PData(2).Size(1), PData(2).Size(2),dop_numen);

% Specify Transmit waveform structure.  
TW(1).type = 'parametric';
TW(1).Parameters = [30,14,2,1];% TW(1).Parameters = [45,20,2,1];%  % TW(1).Parameters = [36,17,2,1]; % 2.5 MHz

% Set up transmit delays in TX structure.
TX.waveform = 1;
TX.Origin = [0,0,0];            % set origin to 0,0,0 for flat focus.
TX.focus = -SFormat.radius;     % set focus to negative for concave TX.Delay profile.
TX.Steer = [0,0];
TX.Apod = ones(1,64);  % set TX.Apod for 64 elements
TX.Delay = computeTXDelays(TX);

% Specify TGC Waveform structure.
TGC(1).CntrlPts = [500,590,650,710,770,830,890,950];
TGC(1).rangeMax = SFormat(1).endDepth;
TGC(1).Waveform = computeTGCWaveform(TGC(1));

% Specify Receive structure arrays
maxAcqLength = sqrt(SFormat(1).endDepth^2 + (Trans.numelements*Trans.spacing)^2) - SFormat(1).startDepth;
wlsPer128 = 128/(4*2); % wavelengths in 128 samples for 4 samplesPerWave
Receive = repmat(struct('Apod', ones(1,Trans.numelements), ...
                        'startDepth', SFormat(1).startDepth, ...
                        'endDepth', SFormat(1).startDepth + wlsPer128*ceil(maxAcqLength/wlsPer128), ...
                        'TGC', 1, ...
                        'bufnum', 1, ...
                        'framenum', 1, ...
                        'acqNum', 1, ...
                        'samplesPerWave', 4, ...
                        'mode', 0, ...
                        'InputFilter',[0.0036,0.0127,0.0066,-0.0881,-0.2595,0.6494], ...
                        'callMediaFunc', 0),1,Nframe);
                                       
% Set receive struct for Bmode
for i = 1:Nframe
    Receive(i).framenum = i;
end

% Specify Recon structure array. 
Recon(1) = struct('senscutoff', 0.5, ...
               'pdatanum', 1, ...
               'rcvBufFrame',-1, ...
               'IntBufDest', [1,1], ...
               'ImgBufDest', [1,-1], ...
               'RINums', 1);
      

% Define ReconInfo structures.
ReconInfo = struct('mode', 0, ... 
                   'txnum', 1, ...
                   'rcvnum', 1, ...
                   'regionnum', 0);
               

% Specify Process structure array.
pers = 10;
Process(1).classname = 'Image';
Process(1).method = 'imageDisplay';
Process(1).Parameters = {'imgbufnum',1,...   % number of buffer to process.
                         'framenum',-1,...   % (-1 => lastFrame)
                         'pdatanum',2,...    % number of PData structure to use
                         'norm',1,...        % normalization method(1 means fixed)
                         'pgain',2.0,...            % pgain is image processing gain
                         'persistMethod','simple',...
                         'persistLevel',pers,...
                         'interp',0,...      % method of interpolation (1=4pt interp)
                         'compression',0.5,...      % X^0.5 normalized to output word size
                         'reject',2,...      % reject level 
                         'mappingMode','full',...
                         'extDisplay', 1, ...
                         'display',1,...      % display image after processing
                         'displayWindow',1};


Process(2).classname = 'External';
Process(2).method = 'trackMotion';
Process(2).Parameters = {'srcbuffer','image',... 
                         'srcbufnum',1,...
                         'srcframenum',-1,...  
                         'dstbuffer','none'};
EF(1).Function = text2cell('%EF#1');

Process(3).classname = 'External';
Process(3).method = 'checkGating';
Process(3).Parameters = {};
EF(2).Function = text2cell('%EF#2');

Process(4).classname = 'External';
Process(4).method = 'preGating';
Process(4).Parameters = {'srcbuffer','image',...
                         'srcbufnum',1,...
                         'srcframenum',-1,... 
                         'dstbuffer','none'};
EF(3).Function = text2cell('%EF#3');

Process(5).classname = 'External';
Process(5).method = 'processListening';
Process(5).Parameters = {'srcbuffer','receive',...
                         'srcbufnum',1,...
                         'dstbuffer','none'};
EF(4).Function = text2cell('%EF#4');

Process(6).classname = 'External';
Process(6).method = 'postGating';
Process(6).Parameters = {'srcbuffer','image',...
                         'srcbufnum',1,...
                         'srcframenum',-1,... 
                         'dstbuffer','none'};
EF(5).Function = text2cell('%EF#5');

Process(7).classname = 'External';
Process(7).method = 'postDoppler';
Process(7).Parameters = {'srcbuffer','inter',...
                         'srcbufnum',2,...
                         'srcframenum',1,... 
                         'dstbuffer','none'};
EF(6).Function = text2cell('%EF#6');

% Specify SeqControl structure arrays.
nsc = 1;
SeqControl(nsc).command = 'jump'; 
SeqControl(1).argument = 1;
SeqControl(2).command = 'timeToNextAcq';
SeqControl(2).argument = 220; 
SeqControl(3).command = 'timeToNextAcq'; 
SeqControl(3).argument = t_frame;
SeqControl(4).command = 'returnToMatlab';
SeqControl(5).command = 'triggerOut';
SeqControl(6).command = 'sync';
SeqControl(6).argument = 10E6;
SeqControl(7).command = 'noop';
SeqControl(7).argument = 10000;
SeqControl(8).command = 'pause'; 
SeqControl(8).condition = 'extTrigger';
SeqControl(8).argument = 17;
nsc = 9; % nsc is count of SeqControl objects

%% Specify Event structure arrays.
n = 1;
Event(n).info = 'Empty event'; 
Event(n).tx = 0;         % no transmit
Event(n).rcv = 0;        % no rcv
Event(n).recon = 0;      % reconstruction
Event(n).process = 0;    % process
Event(n).seqControl = 0;
n = n+1;

Event(n).info = 'Empty event'; 
Event(n).tx = 0;         % no transmit
Event(n).rcv = 0;        % no rcv
Event(n).recon = 0;      % reconstruction
Event(n).process = 0;    % process
Event(n).seqControl = 0;
n = n+1;

for i = 1:Nframe
    Event(n).info = 'Empty event'; 
    Event(n).tx = 0;         % no transmit
    Event(n).rcv = 0;        % no rcv
    Event(n).recon = 0;      % reconstruction
    Event(n).process = 0;    % process
    Event(n).seqControl = 0;
    n = n+1;
    
    Event(n).info = 'Acquire full aperture';
    Event(n).tx = 1;         % use 1st TX structure.
    Event(n).rcv = i; 
    Event(n).recon = 0;      % no reconstruction.
    Event(n).process = 0;    % no processing
    Event(n).seqControl = [3,nsc]; % time between frames & transferToHostuse
       SeqControl(nsc).command = 'transferToHost';
       nsc = nsc + 1;
    n = n+1;
    
    Event(n).info = 'recon and process'; 
    Event(n).tx = 0;         % no transmit
    Event(n).rcv = 0;        % no rcv
    Event(n).recon = 0;      % reconstruction
    Event(n).process = 0;    % process
    Event(n).seqControl = 0;
    n = n+1;
        
    Event(n).info = 'recon and process'; 
    Event(n).tx = 0;         % no transmit
    Event(n).rcv = 0;        % no rcv
    Event(n).recon = 1;      % reconstruction
    Event(n).process = 0;    % process
    Event(n).seqControl = 0;
    n = n+1;
    
    Event(n).info = 'motion tracking'; 
    Event(n).tx = 0;         % no transmit
    Event(n).rcv = 0;        % no rcv
    Event(n).recon = 0;      % reconstruction
    Event(n).process = 2;    % process
    Event(n).seqControl = 0;
    n = n+1;
      
    Event(n).info = 'display event'; 
    Event(n).tx = 0;         % no transmit
    Event(n).rcv = 0;        % no rcv
    Event(n).recon = 0;      % reconstruction
    Event(n).process = 1;    % process
    Event(n).seqControl = 0;
    n = n+1;
end


Event(n).info = 'Call conditional gating event'; 
Event(n).tx = 0;         % no transmit
Event(n).rcv = 0;        % no rcv
Event(n).recon = 0;      % reconstruction
Event(n).process = 0;    % process
Event(n).seqControl = nsc;
    SeqControl(nsc).command = 'call';
    SeqControl(nsc).argument = n+4; %this event should be event called 'condition' number
    nsc = nsc+1;
n = n+1;

Event(n).info = 'jump back'; 
Event(n).tx = 0;         % no transmit
Event(n).rcv = 0;        % no rcv
Event(n).recon = 0;      % reconstruction
Event(n).process = 0;    % process
Event(n).seqControl = 1;
n = n+1;

Event(n).info = 'Send trigger out'; 
Event(n).tx = 0;         % no transmit
Event(n).rcv = 0;        % no rcv
Event(n).recon = 0;      % reconstruction
Event(n).process = 0;    % process
Event(n).seqControl = 5;
n = n+1;

Event(n).info = 'Jump back to start'; 
Event(n).tx = 0;         % no transmit
Event(n).rcv = 0;        % no rcv
Event(n).recon = 0;      % reconstruction
Event(n).process = 0;    % process
Event(n).seqControl = 1;
n = n+1;

Event(n).info = 'Condition'; 
Event(n).tx = 0;         % no transmit
Event(n).rcv = 0;        % no rcv
Event(n).recon = 0;      % reconstruction
Event(n).process = 3;    % process
Event(n).seqControl = 0;
n = n+1;

Event(n).info = 'jump to startEvent'; 
Event(n).tx = 0;         % no transmit
Event(n).rcv = 0;        % no rcv
Event(n).recon = 0;      % reconstruction
Event(n).process = 0;    % process
Event(n).seqControl = nsc;
    SeqControl(nsc).command = 'rtn';
n = n+1;


% User specified UI Control Elements
% - Sensitivity Cutoff
n_ui = 1;

UI(n_ui).Control = {'UserB1','Style','VsToggleButton','Label','z tracking'};
UI(n_ui).Callback = text2cell('%CB#1');
n_ui = n_ui + 1;

UI(n_ui).Control = {'UserB2','Style','VsToggleButton','Label','x tracking'};
UI(n_ui).Callback = text2cell('%CB#L');
n_ui = n_ui + 1;

UI(n_ui).Control =  {'UserC3','Style','VsSlider','Label','min mc x',...
                  'SliderMinMaxVal',[0.5,1,mot_min_mc_x],...
                  'SliderStep',[0.025,0.1],'ValueFormat','%3.2f'};
UI(n_ui).Callback = text2cell('%CB#M');
n_ui = n_ui + 1;

UI(n_ui).Control = {'UserC1','Style','VsToggleButton','Label','z comp.'};
UI(n_ui).Callback = text2cell('%CB#N');
n_ui = n_ui + 1;
UI(n_ui).Statement = 'set(findobj(''String'', ''z comp.''), ''Value'', mot_comp_on_z);';n_ui = n_ui + 1;

UI(n_ui).Control = {'UserC2','Style','VsToggleButton','Label','x comp.'};
UI(n_ui).Callback = text2cell('%CB#O');
n_ui = n_ui + 1;
UI(n_ui).Statement = 'set(findobj(''String'', ''x comp.''), ''Value'', mot_comp_on_x);';n_ui = n_ui + 1;

UI(n_ui).Control = {'UserB3','Style','VsToggleButton','Label','Plot motion'};
UI(n_ui).Callback = text2cell('%CB#2');
n_ui = n_ui + 1;

% UI(n_ui).Control =  {'UserB3','Style','VsSlider','Label','Track thresh.',...
%                   'SliderMinMaxVal',[0.1,0.9,mot_thresh_z],...
%                   'SliderStep',[0.025,0.1],'ValueFormat','%1.2f'};
% UI(n_ui).Callback = text2cell('%CB#3');
% n_ui = n_ui + 1;

UI(n_ui).Statement = '[result,hv] = setTpcProfileHighVoltage(5,1);';n_ui = n_ui + 1; 
UI(n_ui).Statement = 'hv1Sldr = findobj(''Tag'',''hv1Sldr'');'; n_ui = n_ui + 1;
UI(n_ui).Statement = 'set(hv1Sldr,''Value'',hv);'; n_ui = n_ui + 1;
UI(n_ui).Statement = 'hv1Value = findobj(''Tag'',''hv1Value'');'; n_ui = n_ui + 1;
UI(n_ui).Statement = 'set(hv1Value,''String'',num2str(hv,''%.1f''));';n_ui = n_ui + 1;

% UI(n_ui).Control =  {'UserC3','Style','VsSlider','Label','ROI start (mm)',...
%                   'SliderMinMaxVal',[0,120,mot_start_z*1E3],...
%                   'SliderStep',[0.025,0.1],'ValueFormat','%3.0f'};
% UI(n_ui).Callback = text2cell('%CB#5');
% n_ui = n_ui + 1;
% 
% UI(n_ui).Control =  {'UserC2','Style','VsSlider','Label','ROI length (mm)',...
%                   'SliderMinMaxVal',[5,60,mot_length_z*1E3],...
%                   'SliderStep',[0.025,0.1],'ValueFormat','%3.0f'};
% UI(n_ui).Callback = text2cell('%CB#6');
% n_ui = n_ui + 1;
% 
% UI(n_ui).Control =  {'UserC1','Style','VsSlider','Label','ROI width (mm)',...
%                   'SliderMinMaxVal',[1,40,mot_width_z*1E3],...
%                   'SliderStep',[0.025,0.1],'ValueFormat','%3.0f'};
% UI(n_ui).Callback = text2cell('%CB#7');
% n_ui = n_ui + 1;

UI(n_ui).Control =  {'UserB7','Style','VsPushButton','Label','Save'};
UI(n_ui).Callback = text2cell('%CB#8');
n_ui = n_ui + 1;

UI(n_ui).Control =  {'UserA2','Style','VsSlider','Label','Ray focus (mm)',...
                  'SliderMinMaxVal',[20,end_depth*1E3,ray_focus],...
                  'SliderStep',[0.025,0.1],'ValueFormat','%3.0f'};
UI(n_ui).Callback = text2cell('%CB#9');
n_ui = n_ui + 1;

% UI(n_ui).Control =  {'UserB5','Style','VsSlider','Label','Ref position (mm)',...
%                   'SliderMinMaxVal',[40,120,mot_ref_pos_z*1E3],...
%                   'SliderStep',[0.025,0.1],'ValueFormat','%1.2f'};
% UI(n_ui).Callback = text2cell('%CB#A');
% n_ui = n_ui + 1;

% UI(n_ui).Control =  {'UserB4','Style','VsPushButton','Label','Set ref.'};
% UI(n_ui).Callback = text2cell('%CB#B');
% n_ui = n_ui + 1;

UI(n_ui).Control = {'UserC8','Style','VsToggleButton','Label','Gate/fire'};
UI(n_ui).Callback = text2cell('%CB#C');
n_ui = n_ui + 1;

UI(n_ui).Control =  {'UserC7','Style','VsSlider','Label','Gat. thresh. (mm)',...
                  'SliderMinMaxVal',[0.1,5,gating_thresh*1E3],...
                  'SliderStep',[0.025,0.1],'ValueFormat','%1.2f'};
UI(n_ui).Callback = text2cell('%CB#D');
n_ui = n_ui + 1;

UI(n_ui).Control =  {'UserC6','Style','VsSlider','Label','Firing PRF (Hz)',...
                  'SliderMinMaxVal',[0.5,5,gating_prf],...
                  'SliderStep',[0.025,0.1],'ValueFormat','%1.2f'};
UI(n_ui).Callback = text2cell('%CB#E');
n_ui = n_ui + 1;

UI(n_ui).Control = {'UserB8','Style','VsToggleButton','Label','Save ite'};
UI(n_ui).Callback = text2cell('%CB#F');
n_ui = n_ui + 1;

% UI(n_ui).Control =  {'UserA1','Style','VsSlider','Label','Img. freq. (MHz)',...
%                   'SliderMinMaxVal',[img_min_freq,img_max_freq, img_freq],...
%                   'SliderStep',[0.025,0.1],'ValueFormat','%1.2f'};
% UI(n_ui).Callback = text2cell('%CB#G');
% n_ui = n_ui + 1;

% UI(n_ui).Control =  {'UserC4','Style','VsSlider','Label','p_burst (ms)',...
%                   'SliderMinMaxVal',[0,20,p_burst*1E3],...
%                   'SliderStep',[0.025,0.1],'ValueFormat','%2.1f'};
% UI(n_ui).Callback = text2cell('%CB#I');
% n_ui = n_ui + 1;

% UI(n_ui).Control =  {'UserB6','Style','VsSlider','Label','ImgData.^n',...
%                   'SliderMinMaxVal',[1,8,mot_exp_z],...
%                   'SliderStep',[1/7,1/7],'ValueFormat','%1.0f'};
% UI(n_ui).Callback = text2cell('%CB#J');
% n_ui = n_ui + 1;

UI(n_ui).Control =  {'UserC5','Style','VsSlider','Label','PAM mode',...
                  'SliderMinMaxVal',[0,2,pam_mode],...
                  'SliderStep',[1/2,1/2],'ValueFormat','%1.0f'};
UI(n_ui).Callback = text2cell('%CB#H');
n_ui = n_ui + 1;

% UI(n_ui).Control = {'UserA2','Style','VsToggleButton','Label','No doppler'};
% UI(n_ui).Callback = text2cell('%CB#K');
% n_ui = n_ui + 1;

UI(n_ui).Control = {'UserB4','Style','VsToggleButton','Label','Focus shift'};
UI(n_ui).Callback = text2cell('%CB#P');
n_ui = n_ui + 1;

UI(n_ui).Control = {'UserA1','Style','VsToggleButton','Label','Flip'};
UI(n_ui).Callback = text2cell('%CB#Q');
n_ui = n_ui + 1;

% Specify factor for converting sequenceRate to frameRate.
frameRateFactor = F_display ;

% Save all the structures to a .mat file.
save(filename);
VSX_gilles_python_display
return

%% External functions
%EF#1
trackMotion(ImgData)

socket = evalin('base', 'socket');
scale_motion = evalin('base', 'scale_motion');
scale_time = evalin('base', 'scale_time');
mot_track_plot = evalin('base', 'mot_track_plot');
mot_track_on_x = evalin('base', 'mot_track_on_x');
mot_track_on_z = evalin('base', 'mot_track_on_z');

persistent tx tz
persistent roi_prev_x

msg_format = 'X%06dZ%06dT%08d\n';
msg_x = 0;
msg_z = 0;
msg_t = -1000;

if evalin('base', 'bmode_flip')
    ImgData = fliplr(ImgData);
end

img_size = size(ImgData);
img_center = round(img_size(2)/2);

lambda_x = evalin('base', 'lambda*PData(1).pdeltaX');
lambda_z = evalin('base', 'lambda*PData(1).pdeltaZ');

lag_secondary_xc = 20;

if mot_track_on_x
    mot_width_x = round(evalin('base', 'mot_width_x')/lambda_x);
    mot_top_x = round(evalin('base', 'mot_top_x')/lambda_z);
    mot_left_x = round(evalin('base', 'mot_left_x')/lambda_x + img_center);
    mot_height_x = round(evalin('base', 'mot_height_x')/lambda_z);
    mot_vect_x = evalin('base', 'mot_vect_x');
    mot_max_dx = evalin('base', 'mot_max_dx');
    mot_ref_x = evalin('base', 'mot_ref_x');
    mot_lag_x = evalin('base', 'mot_lag_x');
    mot_time_x = evalin('base', 'mot_time_x');
    mot_mc_vect_x = evalin('base', 'mot_mc_vect_x');
    mot_min_mc_x = evalin('base', 'mot_min_mc_x');
    
    interp_coeff = 8;
    wl_to_m = lambda_x/interp_coeff;    
    
    roi = sum(ImgData(mot_top_x:mot_top_x+mot_height_x,...
                      mot_left_x:mot_left_x+mot_width_x), 1);
    N = 1:length(roi);
    interp_N = 1:1/interp_coeff:N(end);
    roi = interp1(N, roi, interp_N, 'pchip');
    
    lag = round(mot_lag_x*mot_width_x*interp_coeff);
    
    if evalin('base', 'mot_ref_x_update')
        disp('Reference updated.')
        assignin('base', 'mot_ref_x', roi);
        roi_prev_x = roi;
        assignin('base', 'mot_ref_x_update', 0);
    end
    
    if isempty(mot_ref_x)
        disp('Need reference position set!')
    else
        mmm = min([mot_ref_x, roi]);
        xcorr_vect = xcorr(mot_ref_x-mmm, roi-mmm, lag, 'coeff');
        [mc,dt] = max(xcorr_vect);
        mot_mc_vect_x = [mot_mc_vect_x, mc];
        if mc > mot_min_mc_x
            current_x = wl_to_m*(dt-lag-1);
            if ~isempty(mot_vect_x)
                if abs(current_x-mot_vect_x(end)) < mot_max_dx
                    mot_vect_x = [mot_vect_x, current_x];
                else
                    disp(['dx=',num2str(1E3*abs(current_x-mot_vect_x(end))),'mm, max displacement violation, using last x value instead']);
                    mot_vect_x = [mot_vect_x, mot_vect_x(end)];
                end
            else
                mot_vect_x = [mot_vect_x, current_x];
            end
        else
            mmm = min([roi_prev_x, roi]);
            xcorr_vect = xcorr(roi_prev_x-mmm, roi-mmm, lag_secondary_xc, 'coeff');
            [mc,dt] = max(xcorr_vect);
            if mc > 0.9
                current_x = wl_to_m*(dt-lag_secondary_xc-1) + mot_vect_x(end);
                if ~isempty(mot_vect_x)
                    if abs(current_x-mot_vect_x(end)) < mot_max_dx
                        mot_vect_x = [mot_vect_x, current_x];
                    else
                        disp(['dx=',num2str(1E3*abs(current_x-mot_vect_x(end))),'mm, max displacement violation, using last x value instead']);
                        mot_vect_x = [mot_vect_x, mot_vect_x(end)];
                    end
                else
                    mot_vect_x = [mot_vect_x, current_x];
                end
            else
                mot_vect_x = [mot_vect_x, mot_vect_x(end)];
            end
        end
        if isempty(mot_time_x)
            mot_time_x = [mot_time_x, 0.0];
            tx = clock;
        else
            mot_time_x = [mot_time_x, etime(clock, tx)];
        end
        if evalin('base', 'mot_comp_on_x')
            msg_x = round(mot_vect_x(end)*scale_motion);
            msg_t = round(mot_time_x(end)*scale_time);
        end
        if mot_track_plot
            figure(300)
            plot(roi)
            hold on
            plot(mot_ref_x, '-.r')
            hold off
            
            figure(301)
            yyaxis left
            plot(mot_vect_x*1000.)
            ylabel('motion (mm)')
            
            yyaxis right
            plot(mot_mc_vect_x)
            ylabel('max correlation coeff.');
        end
    end
    roi_prev_x = roi;
else
    mot_vect_x = [];
    mot_time_x = [];
    mot_mc_vect_x = [];
end

if mot_track_on_z
    mot_width_z = evalin('base', 'mot_width_z')/lambda_x;
    mot_length_z = evalin('base', 'mot_length_z')/lambda_z;
    mot_start_z = evalin('base', 'mot_start_z')/lambda_z;
    mot_vect_z = evalin('base', 'mot_vect_z');
    mot_time_z = evalin('base', 'mot_time_z');
    mot_exp_z = evalin('base', 'mot_exp_z');
    mot_thresh_z = evalin('base', 'mot_thresh_z');
    mot_max_dz = evalin('base', 'mot_max_dz');
    mot_ref_pos_z = evalin('base', 'mot_ref_pos_z');

    interp_coeff = 8;
    wl_to_m = lambda_z/interp_coeff;
    if ~isempty(mot_vect_z)
        mot_start_z = mot_start_z + round((mot_vect_z(end)-mot_vect_z(1))/lambda_z); 
    end
    roi = sum(ImgData(mot_start_z:mot_start_z+mot_length_z,...
                      img_center-round(mot_width_z/2):img_center+round(mot_width_z/2)), 2);
    if mot_exp_z > 1
        roi = roi.^mot_exp_z;
    end
    roi = roi./max(roi)-mot_thresh_z;
    N = 1:length(roi);
    interp_N = 1:1/interp_coeff:N(end);
    roi = interp1(N, roi, interp_N, 'pchip');
    zc = find(roi(:).*circshift(roi(:), [-1 0]) <= 0);
    if length(zc) == 0
        zc = 0;
    else
        zc = zc(1);
    end
    current_z = mot_start_z*lambda_z+zc*wl_to_m;
    if ~isempty(mot_vect_z)
        if abs(current_z-mot_vect_z(end)) < mot_max_dz
            mot_vect_z = [mot_vect_z, current_z];
        else
            disp(['dz=',num2str(1E3*abs(current_z-mot_vect_z(end))),'mm, max displacement violation, using last z value instead']);
            mot_vect_z = [mot_vect_z, mot_vect_z(end)];
        end
    else
        mot_vect_z = [mot_vect_z, current_z];
    end
    if isempty(mot_time_z)
        mot_time_z = [mot_time_z, 0.0];
        tz = clock;
    else
        mot_time_z = [mot_time_z, etime(clock, tz)];
    end
    if evalin('base', 'mot_comp_on_z')
        msg_z = round((mot_vect_z(end)-mot_ref_pos_z)*scale_motion);
        msg_t = round(mot_time_z(end)*scale_time);
    end
    if mot_track_plot
        abss = 1000.*linspace(mot_start_z,mot_start_z+mot_length_z, length(roi))*lambda_z;
        figure(200)
        plot(abss, roi+mot_thresh_z)
        hold on
        plot([abss(1) abss(end)], mot_thresh_z*[1 1], '--k')
        plot(current_z*1000, mot_thresh_z, 'xr', 'MarkerSize', 16, 'LineWidth', 2)
        plot([mot_ref_pos_z, mot_ref_pos_z].*1000., [0, 1], ':m')
        hold off
        xlabel('Position (mm)')
        figure(201)
        plot(mot_time_z, (mot_vect_z-mot_ref_pos_z)*1000.)
        xlabel('Time (s)')
        ylabel('motion (mm)')
    end
else
    mot_vect_z = [];
    mot_time_z = [];
end

msg = sprintf(msg_format, [msg_x, msg_z, msg_t]);
fwrite(socket, msg);

assignin('base', 'mot_vect_x', mot_vect_x);
assignin('base', 'mot_time_x', mot_time_x);
assignin('base', 'mot_vect_z', mot_vect_z);
assignin('base', 'mot_time_z', mot_time_z);
assignin('base', 'mot_mc_vect_x', mot_mc_vect_x);
return
%EF#1

%EF#2
checkGating()
gating_bool = evalin('base', 'gating_bool');
gating_delay = evalin('base', 'gating_delay');
gating_thresh = evalin('base', 'gating_thresh');
gating_prf = evalin('base', 'gating_prf');
last_firing_clock = evalin('base', 'last_firing_clock');
mot_vect_x = evalin('base', 'mot_vect_x');
mot_vect_z = evalin('base', 'mot_vect_z');
mot_ref_pos_z = evalin('base', 'mot_ref_pos_z');
evt_firing = evalin('base', '2+6*Nframe+3');
gating_num = evalin('base', 'gating_num');

Control.Command = 'set';
Control.Parameters = {'Parameters', 1, 'startEvent', 1};

% First check if gating or firing is needed
if gating_bool
    % Then if there is any motion data
    if ~isempty(mot_vect_x) || ~isempty(mot_vect_z)
        % Then if it is time to fire
%         dt = etime(clock, last_firing_clock);
%         if dt >= 1./gating_prf
            % Then check position
            if ~isempty(mot_vect_x)
                dx = abs(mot_vect_x(end));
            else
                dx = 0;
            end
            if ~isempty(mot_vect_z)
                dz = abs(mot_vect_z(end)-mot_ref_pos_z);
            else
                dz = 0;
            end
            dd = sqrt(dx^2 + dz^2);
            if dd <= gating_thresh
%                 disp(['Firing at dt=', num2str(dt), 's, dx=', num2str(dx*1E3), ', dz=', num2str(dz*1E3), 'mm and listening...'])
                Control.Parameters = {'Parameters', 1, 'startEvent', evt_firing};
                assignin('base', 'last_firing_clock', clock);
                assignin('base', 'gating_num', gating_num+1);
                assignin('base', 'just_gated', 1);
            end
%         end
    else % If no motion data then fire without gating
%         dt = etime(clock, last_firing_clock);
%         if dt >= 1./gating_prf
%             disp(['Firing without gating at dt=', num2str(dt), 's and listening...'])
            Control.Parameters = {'Parameters', 1, 'startEvent', evt_firing};
            assignin('base', 'last_firing_clock', clock);
            assignin('base', 'gating_num', gating_num+1);
            assignin('base', 'just_gated', 1);
%         end
    end
end

assignin('base', 'Control', Control);
return
%EF#2

%EF#3
preGating(ImgData)

nz = evalin('base', 'PData(1).Size(1)');
assignin('base', 'img_pre_gating', ImgData(1:nz,:));

return
%EF#3

%EF#4
processListening(Rdata)
Receive = evalin('base', 'Receive');
lstn_rcv = evalin('base', 'lstn_rcv');
lstn_time = evalin('base', 'lstn_time');
sampling_freq = 4*evalin('base', 'Trans.frequency')*1E6;
gating_num = evalin('base', 'gating_num');
pam_mode = evalin('base', 'pam_mode');
dop_numen = evalin('base', 'dop_numen');
RFData = double(Rdata(1:Receive(end-dop_numen).endSample,:,end)); 
assignin('base', 'RFData', RFData);

% Doppler RF
RFDop = double(Rdata(Receive(end-dop_numen).endSample+1:Receive(end).endSample,:,end)); 
assignin('base', 'RFDop', RFDop);

t = (1:length(RFData(:,1)))/sampling_freq;

if pam_mode >= 1
    figure(100)
    plot(t*1E3, RFData(:,32))
    title('Center element')
    axis tight
    xlabel('Time (ms)')
% elseif pam_mode == 2
%     tic
%     freqs_bb = evalin('base', 'freqs_bb');
%     c0 = evalin('base', 'c0');
%     x_pam = evalin('base', 'x_pam');
%     z_pam = evalin('base', 'z_pam');
%     sampling_freq = 4*evalin('base', 'Trans.frequency')*1E6;
%     toc
end

last_firing_clock = evalin('base', 'last_firing_clock');
t_wait = (evalin('base', 'p_burst')+0.002)*20 - etime(clock, last_firing_clock)
if t_wait > 0
    pause(t_wait)
end

return
%EF#4

%EF#5
postGating(ImgData)

persistent n_img

if isempty(n_img)
    n_img = 1;
end

if evalin('base', 'just_gated')
    img_post_gating = evalin('base', 'img_post_gating');
    nz = evalin('base', 'PData(1).Size(1)');
    if n_img <= evalin('base', 'Nf_post')
        img_post_gating(:,:,n_img) = ImgData(1:nz,:);
        assignin('base', 'img_post_gating', img_post_gating);
        n_img = n_img+1;
    else
        if evalin('base', 'save_ite')
            gating_num = evalin('base', 'gating_num');
            img_pre_gating = evalin('base', 'img_pre_gating');
            RFData = evalin('base', 'RFData');
            iq_Dop_gating = evalin('base', 'iq_Dop_gating');
            RFDop = evalin('base', 'RFDop');
            parfeval(@saveImgAndRFDop,0, img_pre_gating, img_post_gating, RFData,iq_Dop_gating, RFDop, ...
                                      ['ite_', num2str(gating_num)]);
        end
        n_img = 1;
        assignin('base', 'just_gated', 0);
    end
end

return
%EF#5

%EF#6
postDoppler(IQData)

assignin('base', 'iq_Dop_gating', reshape(IQData,[size(IQData,1),size(IQData,2),size(IQData,4)]));

%EF#6

%% UI callback routines
%CB#1
assignin('base','mot_track_on_z',UIState);
return
%CB#1

%CB#2
assignin('base','mot_track_plot',UIState);
return
%CB#2

%CB#3
assignin('base', 'mot_thresh_z', UIValue)
return
%CB#3

%CB#5
mot_start_z = round(UIValue)*1E-3;
assignin('base', 'mot_start_z', mot_start_z)

evalin('base', 'PythonDisplay.draw_roi([-mot_width_z/2, mot_start_z], mot_width_z, mot_length_z, "blue")');
return
%CB#5

%CB#6
mot_length_z = round(UIValue)*1E-3;
assignin('base', 'mot_length_z', mot_length_z)

evalin('base', 'PythonDisplay.draw_roi([-mot_width_z/2, mot_start_z], mot_width_z, mot_length_z, "blue")');
return
%CB#6

%CB#7
mot_width_z = round(UIValue)*1E-3;
assignin('base', 'mot_width_z', mot_width_z)

evalin('base', 'PythonDisplay.draw_roi([-mot_width_z/2, mot_start_z], mot_width_z, mot_length_z, "blue")');
return
%CB#7

%CB#8
mot_vect_z = evalin('base', 'mot_vect_z');
mot_time_z = evalin('base', 'mot_time_z');
mot_filename=['Data/motion_' datestr(now,30)];
save(mot_filename,'mot_vect_z', 'mot_time_z');

img_pre_gating = evalin('base', 'img_pre_gating');
img_post_gating = evalin('base', 'img_post_gating');
RFData = evalin('base', 'RFData');
bh_fname=['Data/BH_manual_save_', datestr(now,30)];
save(bh_fname, 'img_pre_gating', 'img_post_gating', 'RFData');
disp('Motion and BG data saved in: Data/')
return
%CB#8

%CB#9
if strcmp(get(hObject,'style'),'slider')%if using slider
    ray_focus = round(get(hObject,'Value'));
else
    ray_focus = round(str2double(get(hObject,'String'))); %if entering text
end
TX = evalin('base', 'TX');
n_ray = evalin('base', 'n_ray');
lambda = evalin('base', 'lambda');

for n = 1:n_ray
    TX(n).focus = ray_focus*1E-3/lambda;
    TX(n).Delay = computeTXDelays(TX(n));
end
disp(ray_focus)
assignin('base', 'TX', TX);
Control.Command = 'update&Run';
Control.Parameters = {'TX'};
assignin('base', 'Control', Control);
evalin('base', 'runAcq(Control)');
return
%CB#9

%CB#A
mot_ref_pos_z = UIValue*1E-3;
assignin('base', 'mot_ref_pos_z', mot_ref_pos_z)
return
%CB#A

%CB#B
mot_vect_z = evalin('base', 'mot_vect_z');

if ~isempty(mot_vect_z)
    mot_ref_pos_z = mot_vect_z(end);
    assignin('base', 'mot_ref_pos_z', mot_ref_pos_z);
    mot_width_z = evalin('base', 'mot_width_z');
    mot_length_z = evalin('base', 'mot_length_z');
    mot_start_z = evalin('base', 'mot_start_z');
    lambda = evalin('base', 'lambda');
    pos_sldr = findobj('Tag', ['UserB5' 'Slider']);
    set(pos_sldr,'Value', mot_ref_pos_z*1E3)
    pos_val = findobj('Tag', ['UserB5' 'Edit']);
    set(pos_val, 'String', num2str(mot_ref_pos_z*1E3, '%3.2f'));
    h_box_ref_poss.YData = [mot_ref_pos_z, mot_ref_pos_z]*1E3;
end
return
%CB#B

%CB#C
assignin('base', 'gating_bool', UIState);
assignin('base', 'gating_num', 0);
return
%CB#C

%CB#D
assignin('base', 'gating_thresh', UIValue*1E-3);
return
%CB#D

%CB#E
assignin('base', 'gating_prf', UIValue);
return
%CB#E

%CB#F
assignin('base', 'save_ite', UIState);
disp(UIState)
return
%CB#F

%CB#G % !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
Receive = evalin('base', 'Receive');
TW = evalin('base', 'TW');
img_valid_tw = evalin('base', 'img_valid_tw');
img_valid_tw_freq = evalin('base', 'img_valid_tw_freq');

if strcmp(get(hObject,'style'),'slider')%if using slider
    img_freq = get(hObject,'Value');
else
    img_freq = str2double(get(hObject,'String')); %if entering text
end

[~, tw_i] = min(abs(img_valid_tw_freq-img_freq))
[N,~] = size(TW(1).Parameters);
tw_freq = img_valid_tw_freq(tw_i)
for i=1:N
    TW(1).Parameters(i,:) = img_valid_tw(tw_i,:);
end

assignin('base', 'TW', TW);
Control.Command = 'update&Run';
Control.Parameters = {'TX', 'TW', 'Event'};
assignin('base', 'Control', Control);
return
%CB#G

%CB#H
if strcmp(get(hObject,'style'),'slider')%if using slider
    pam_mode = round(get(hObject,'Value'));
else
    pam_mode = round(str2double(get(hObject,'String'))); %if entering text
end

if pam_mode == 2
    RFData = evalin('base', 'RFData');
    if ~isempty(RFData)
        tic
        figure(101)
        plot(RFData(:,32))
        title('Center element')
        freqs_bb = evalin('base', 'freqs_bb');
        c0 = evalin('base', 'c0');
        x_pam = evalin('base', 'x_pam');
        z_pam = evalin('base', 'z_pam');
        sampling_freq = 4*evalin('base', 'Trans.frequency')*1E6;
        t = (1:length(RFData(:,1)))/sampling_freq;
        L = length(RFData(:,1));
        if mod(L,2)
            L = L+1;
        end
        f_vect = sampling_freq*(0:(L/2))/L;
        RF_fft = fft(RFData,L,1);
        mag2 = abs(RF_fft/L);
        mag1 = mag2(1:L/2+1, :);
        mag1(2:end-1,:) = 2*mag1(2:end-1,:);

        arg2 = angle(RF_fft);
        arg1 = arg2(1:L/2+1,:);
        arg1(2:end-1,:) = arg1(2:end-1,:);
        img = zeros(length(x_pam), length(z_pam), length(freqs_bb));
        if evalin('base', 'pam_fast')
            for i = 1:length(freqs_bb)
                img(:,:,i) = abs(as_pam_fast(mag1,arg1,f_vect,x_pam,z_pam,freqs_bb(i),c0)).^2;
            end
            img_frame = sum(img,3)/length(freqs_bb);
            imgpost = evalin('base', 'img_post_gating');
            imgpre = evalin('base', 'img_pre_gating');
            bmode_frames = cat(3, imgpre, imgpost);
            PData = evalin('base', 'PData');
            x_bmode = evalin('base', 'x_bmode');
            z_bmode = evalin('base', 'z_bmode');
            lambda = evalin('base', 'lambda');
            figure(501)
            Resource = evalin('base', 'Resource');
            bmode_display_size = evalin('base', 'bmode_display_size');
            disp_pos = Resource.DisplayWindow(1).Position;
            disp_pos(3) = bmode_display_size(1) + 80;
            disp_pos(4) = bmode_display_size(2) - 80;
            disp_pos(1) = 2*disp_pos(1)+disp_pos(3);
            set(gcf, 'Position', disp_pos);
            img_to_display = imagesc(x_bmode*1E3, z_bmode*1E3, bmode_frames(:,:,1).^0.5,'CDataMapping','scaled');
%             img_to_display = imagesc(bmode_frames(:,:,1),'CDataMapping','scaled');
            ax1 = gca;
            colormap(ax1, gray(128).^0.5);
            xlabel('x (mm)')
            ylabel('z (mm)')
            hold all
            axis equal tight
            ax2 = axes;
            img_bb = imagesc(ax2,x_pam*1E3, z_pam*1E3, img_frame');
            linkaxes([ax1 ax2])
            colormap(ax2, 'hot');
            alpha(img_bb, 'color')
            alpha(img_bb, img_frame'./max(max(img_frame))*0.8);
            ax2.Visible = 'off';
            ax2.XTick = [];
            ax2.YTick = [];
            colorbar(ax2, 'Position',[.88 .11 .04 .815])
            hold off
            hbg = uicontrol('style','slider','units','normalized',...
                            'SliderStep', [1/5, 1/5],...
                            'position',[0.6 0.01 0.3 0.02]);
            hbg_txt = uicontrol('style','text','units','normalized','position',[0.6 0.03 0.3 0.02]);
            set(hbg_txt, 'String', 'Frame: 1');
            addlistener(hbg,'ContinuousValueChange',@(hObject, event) change_background(hObject,event,5,img_to_display, bmode_frames,hbg_txt));
            hfg = uicontrol('style','slider','units','normalized','Value', 0.8,'position',[0.1 0.01 0.3 0.02]);
            hfg_txt = uicontrol('style','text','units','normalized','position',[0.1 0.03 0.3 0.02]);
            set(hfg_txt, 'String', 'PAM alpha');
            addlistener(hfg,'ContinuousValueChange',@(hObject, event) change_forground(hObject,event,img_bb, img_frame));
        end
    end
    toc
end
assignin('base', 'pam_mode', pam_mode);
return
%CB#H

%CB#I
if strcmp(get(hObject,'style'),'slider')%if using slider
    p_burst = get(hObject,'Value')*1E-3;
else
    p_burst = str2double(get(hObject,'String'))*1E-3; %if entering text
end
assignin('base', 'p_burst', p_burst);
return
%CB#I

%CB#J
if strcmp(get(hObject,'style'),'slider')%if using slider
    mot_exp_z = round(get(hObject,'Value'));
else
    mot_exp_z = round(str2double(get(hObject,'String'))); %if entering text
end
assignin('base', 'mot_exp_z', mot_exp_z);
return
%CB#J

%CB#K
TX = evalin('base', 'TX');
n_ray = evalin('base', 'n_ray');
Trans = evalin('base', 'Trans');
if UIState
    TX(n_ray+2).Apod = zeros(Trans.numelements,1)';
else
    TX(n_ray+2).Apod = kaiser(Trans.numelements,2)';
end
assignin('base', 'TX', TX);
Control.Command = 'update&Run';
Control.Parameters = {'TX', 'TW', 'Event'};
assignin('base', 'Control', Control);
return
%CB#K

%CB#L
assignin('base','mot_track_on_x',UIState);
return
%CB#L


%CB#M
if strcmp(get(hObject,'style'),'slider')%if using slider
    mot_min_mc_x = get(hObject,'Value');
else
    mot_min_mc_x = str2double(get(hObject,'String')); %if entering text
end
assignin('base', 'mot_min_mc_x', mot_min_mc_x);
return
%CB#M

%CB#N
assignin('base','mot_comp_on_z',UIState);
return
%CB#N

%CB#O
assignin('base','mot_comp_on_x',UIState);
return
%CB#O

%CB#P
if UIState
    focus_pos_bh = evalin('base', 'focus_pos_bh');
    d = focus_shift;
    assignin('base', 'shifted_focus_pos', focus_pos_bh-[0,d]);
    evalin('base', 'PythonDisplay.add_target(shifted_focus_pos, ''green'', ''sfocus'')');
    evalin('base', 'PythonDisplay.draw_layers(d_water, d_fat)');
    vsx_foc_calc_gui;
else
    evalin('base', 'PythonDisplay.remove_target(''sfocus'')');
    evalin('base', 'PythonDisplay.clear_layers()');
    delete(findobj('tag','UIFOC'));
end

return
%CB#P

%CB#Q
assignin('base', 'bmode_flip', UIState);
if UIState
    evalin('base', 'PythonDisplay.move_target(''focus'', focus_pos_bh_fliped)');
else
    evalin('base', 'PythonDisplay.move_target(''focus'', focus_pos_bh)');
end
return
%CB#Q