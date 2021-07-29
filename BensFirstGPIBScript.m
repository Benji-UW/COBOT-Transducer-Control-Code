% Focal length optimization script

clear all
close all
clc

%% Parameters
% Server
socket = tcpclient('localhost', 50008);
% set(socket,'OutputBufferSize', 1024);
% fopen(socket);
scale_motion = 1E6;
% scale_time = 1E3;

% Speed of sound in water: 1500 m/s
% Focal length of the transducer: 1" or 25.4 mm
% Travel distance of the soundwave: 50.8 mm
% Travel time: 33.867 microseconds
% target_time = (50.8 / 1000) / 1500;

% Experimental focal length: 23.7750 mm (perhaps due to depth of concavity)
target_time = (47.55 / 1000) / 1500;

%% Initialize
agiobj=agi_init(4);

Time.delay = 31.8e-6;
Time.scale = 70e-6;
Volt.coup = 'DC';
Volt.imp = 'FIFT';
Volt.range = 0.05;
Acq.mode = 'Normal';
Acq.Seg = 0;
Acq.SegNum = 0;
Acq.AvgNum = 1;
Trig.mode = 'auto';
Trig.slope = 'NEG';
Trig.source = 'EXT';
Trig.level =0.5;

agi_settings(agiobj,Acq,Time,Volt,Trig)

%settings

%% acquire
zHist = ones(60,2);

for abcd = 1:60
    tic
    t = agi_acquire(agiobj, 0);

    T0 = t.T0;
    dT = t.dT;

    % Process

    l = length(t.signal);

    duration = l * dT;

    splt = (2e-5 - T0) / duration;

    tvect = T0:dT:(T0 + (l-1)*dT);

    % figure(1)
%     plot(tvect, t.signal);
%     hold on
%     xline(target_time);
    % Filter
    signal_trunc = t.signal((round(l * splt)):l);
    sig_filtered = sgolayfilt(signal_trunc, 1, 21);
    tvect_trunc = tvect((l * splt):l);

    % figure(2)

    % plot(tvect_trunc, sig_filtered);

    [m, i] = max(sig_filtered);

    tm = tvect_trunc(i);

    % xline(tm);

    z_mot = (tm - target_time) * 1500;

    z_motT = z_mot * scale_motion;

    % fclose(socket);

    msg_format = 'T%02dZ%09d\n';
    msg = sprintf(msg_format, [mod(abcd,100), round(z_motT)]);

    write(socket, msg);
    
%     zHist(abcd, 1) = z_motT / 1000;
    zHist(abcd, 1) = tm * 1.5;
    if abcd == 1
        zHist(abcd, 2) = toc;
    else
        zHist(abcd, 2) = zHist(abcd - 1, 2) + toc;
    end

end

plot(zHist(:,2), zHist(:,1))
xlabel('time (s)')
ylabel('zDistance, (1mm)')
yline(target_time * 1.5)
yline((target_time * 1.5) + 0.5e-6)
yline((target_time * 1.5) - 0.5e-6)

