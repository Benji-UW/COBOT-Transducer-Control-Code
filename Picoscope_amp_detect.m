%%
% This version of the control code is meant to be operated in
% amplitude-detect mode, this is the case in which the MATLAB script parses
% input from the transducer and detects the strength of the return signal
% and leaves all the other optimization work to the python script :/
clear all;
close all;
clc;

%% Server setup

socket = tcpclient('localhost', 50008);
scale_motion = 1E6;

% Experimental focal length: 23.7750 mm (perhaps due to depth of concavity)
target_time = (47.55 / 1000) / 1500;

%% Picoscope setup

PS5000aConfig;
channelA = ps5000aEnuminfo.enPS5000AChannel.PS5000A_CHANNEL_A;

% Close any existing device objects if the user chooses to do so
if (exist('ps5000aDeviceObj', 'var') && ps5000aDeviceObj.isvalid && ...
        strcmp(ps5000aDeviceObj.status, 'open'))
    
    openDevice = questionDialog(['Device object ps5000aDeviceObj ' ... 
        'has an open connection. ' ...
        'Do you wish to close the connection and continue?'], ...
        'Device Object Connection Open');
    
    if (openDevice == PicoConstants.TRUE)
        
        % Close connection to device.
        disconnect(ps5000aDeviceObj);
        delete(ps5000aDeviceObj);
        
    else

        % Exit script if User selects 'No'.
        return;
        
    end
    
end

% Create and connect to a device object
ps5000aDeviceObj = icdevice('picotech_ps5000a_generic', ''); 
connect(ps5000aDeviceObj);

% I probably don't need to see this but it makes my other script work so
% here is the unit information:
[status.getUnitInfo, unitInfo] = invoke(ps5000aDeviceObj, 'getUnitInfo');
disp(unitInfo);

% Channel setup - we're only using channel A and setting it to 
channelSettings(1).enabled = PicoConstants.TRUE;
channelSettings(1).coupling = ps5000aEnuminfo.enPS5000ACoupling.PS5000A_DC;
channelSettings(1).range = ps5000aEnuminfo.enPS5000ARange.PS5000A_500MV;
channelSettings(1).analogueOffset = 0.0;

channelARangeMv = PicoConstants.SCOPE_INPUT_RANGES(channelSettings(1).range + 1);

numChannels = PicoConstants.SINGLE_SCOPE;

status.setChannelStatus(1) = invoke(ps5000aDeviceObj, ... 
    'ps5000aSetChannel', (0), channelSettings(1).enabled, ...
    channelSettings(1).coupling, channelSettings(1).range, ...
    channelSettings(1).analogueOffset);

% The resolution is set to 8 bits as that seems to suffice for our purposes
[status.setResolution, resolution] = invoke(ps5000aDeviceObj, 'ps5000aSetDeviceResolution', 8);  
maxADCCount = get(ps5000aDeviceObj, 'maxADCValue');

% Trigger setup
triggerGroupObj = get(ps5000aDeviceObj, 'Trigger');
triggerGroupObj = triggerGroupObj(1);

set(triggerGroupObj, 'autoTriggerMs', 10);

channel = ps5000aEnuminfo.enPS5000AChannel.PS5000A_EXTERNAL;
[status.setSimpleTrigger] = invoke(triggerGroupObj, 'setSimpleTrigger', 0, 200, 2);

%% Resolution settings

sampleRate = 3.2e-8; % Sample period, this helps us calculate several other values
% We only really want the 90 us after the trigger, so a lot of data is just
% going down the tubes as far as we're concerned.
samplesPerSecond = 1 / sampleRate;
samplePeriod = 1e-4; % 100 us

% Size of the buffer to collect data from buffer.
overviewBufferSize  = 312500;
segmentIndex        = 0;   
ratioMode           = ps5000aEnuminfo.enPS5000ARatioMode.PS5000A_RATIO_MODE_NONE;

pDriverBufferChA = libpointer('int16Ptr', zeros(overviewBufferSize, 1, ...
    'int16'));

status.setDataBufferChA = invoke(ps5000aDeviceObj, 'ps5000aSetDataBuffer', ...
    channelA, pDriverBufferChA, overviewBufferSize, segmentIndex, ratioMode);

pAppBufferChA = libpointer('int16Ptr', zeros(overviewBufferSize, 1, 'int16'));

streamingGroupObj = get(ps5000aDeviceObj, 'Streaming');
streamingGroupObj = streamingGroupObj(1);

status.setAppDriverBuffersA = invoke(streamingGroupObj, 'setAppAndDriverBuffers', channelA, ...
    pAppBufferChA, pDriverBufferChA, overviewBufferSize);

% Sample rate is defined by the 
set(streamingGroupObj, 'streamingInterval', sampleRate);

set(ps5000aDeviceObj, 'numPreTriggerSamples', 0);
set(ps5000aDeviceObj, 'numPostTriggerSamples', 31250); % 1e4


set(streamingGroupObj, 'autoStop', PicoConstants.FALSE);

downSampleRatio = 1;
downSampleRatioMode = ps5000aEnuminfo.enPS5000ARatioMode.PS5000A_RATIO_MODE_NONE;
 
maxSamples = get(ps5000aDeviceObj, 'numPreTriggerSamples') + ...
    get(ps5000aDeviceObj, 'numPostTriggerSamples');

finalBufferLength = round(1.5 * maxSamples / downSampleRatio);

pBufferChAFinal = libpointer('int16Ptr', zeros(finalBufferLength, ...
    1, 'int16'));


originalPowerSource = invoke(ps5000aDeviceObj, 'ps5000aCurrentPowerSource');

% Start streaming data collection.
[status.runStreaming, sampleInterval, sampleIntervalTimeUnitsStr] = ...
    invoke(streamingGroupObj, 'ps5000aRunStreaming', downSampleRatio, ...
    downSampleRatioMode, overviewBufferSize);

disp('Streaming data...');
fprintf('Click the STOP button to stop capture or wait for auto stop if enabled.\n') 

% Variables to be used when collecting the data:

hasAutoStopOccurred = PicoConstants.FALSE;  % Indicates if the device has stopped automatically.
powerChange         = PicoConstants.FALSE;  % If the device power status has changed.
newSamples          = 0; % Number of new samples returned from the driver.
previousTotal       = 0; % The previous total number of samples.
totalSamples        = 0; % Total samples captured by the device.
startIndex          = 0; % Start index of data in the buffer returned.
hasTriggered        = 0; % To indicate if trigger has occurred.
triggeredAtIndex    = 0; % The index in the overall buffer where the trigger occurred.

time = zeros(overviewBufferSize / downSampleRatio, 1);	% Array to hold time values

status.getStreamingLatestValuesStatus = PicoStatus.PICO_OK; % OK

% Display a 'Stop' button.
[stopFig.h, stopFig.h] = stopButton();             
             
flag = 1; % Use flag variable to indicate if stop button has been clicked (0).
setappdata(gcf, 'run', flag);

% Plot Properties - these are for displaying data as it is collected.


%%
% Collect samples as long as the |hasAutoStopOccurred| flag has not been
% set or the call to |getStreamingLatestValues()| does not return an error
% code (check for STOP button push inside loop).
timeout = 250;
avg_samp_freq = zeros(timeout, 1);
ind = 0;
% zHist = ones(timeout,2);

missed_phases = 0;
tic
while(hasAutoStopOccurred == PicoConstants.FALSE && ...
        status.getStreamingLatestValuesStatus == PicoStatus.PICO_OK && ...
        ind < timeout + 1)
    missed_phases = missed_phases + 1;
    ready = PicoConstants.FALSE;
   
    while (ready == PicoConstants.FALSE)

       status.getStreamingLatestValuesStatus = invoke(streamingGroupObj, 'getStreamingLatestValues'); 
        
       ready = invoke(streamingGroupObj, 'isReady');
       % Give option to abort from here
       flag = getappdata(gcf, 'run');
       drawnow;

       if (flag == 0)
            disp('STOP button clicked - aborting data collection.')
            break;
       end

       drawnow;

    end
    
    % Check for data
    [newSamples, startIndex] = invoke(streamingGroupObj, 'availableData');

    if (newSamples > 0 && startIndex == 0)
        
        % Check if the scope has triggered.
        [triggered, triggeredAt] = invoke(streamingGroupObj, 'isTriggerReady');

        if (triggered == PicoConstants.TRUE)

            % Adjust trigger position as MATLAB does not use zero-based
            % indexing.
            bufferTriggerPosition = triggeredAt + 1;
            
            fprintf('Triggered - index in buffer: %d\n', bufferTriggerPosition);

            hasTriggered = triggered;

            % Set the total number of samples at which the device
            % triggered.
            triggeredAtIndex = totalSamples + bufferTriggerPosition;

        end

        previousTotal   = totalSamples;
        totalSamples    = totalSamples + newSamples;

        % Printing to console can slow down acquisition - use for
        % demonstration.
%         fprintf('Collected %d samples, startIndex: %d total: %d.\n', ...
%           newSamples, startIndex, totalSamples);
        
        % Position indices of data in the buffer(s).
        firstValuePosn = startIndex + 1;
        lastValuePosn = startIndex + newSamples;
        
        % Convert data values to millivolts from the application buffer(s).
        bufferChAmV = adc2mv(pAppBufferChA.Value(firstValuePosn:lastValuePosn), ...
            channelARangeMv, maxADCCount);

        % Process collected data further if required - this example plots
        % the data if the User has selected 'Yes' at the prompt.
        
        % Copy data into the final buffer(s).
%         pBufferChAFinal.Value(previousTotal + 1:totalSamples) = bufferChAmV;        
        
        if (max(bufferChAmV) == 500)% && startIndex == 0)
            LoadTime = toc;
            ind = ind + 1;
            avg_samp_freq(ind) = 1 / LoadTime;
%             
%             Time axis. 
%             Multiply by ratio mode as samples get reduced
%             time = (double(sampleInterval) * double(downSampleRatio)) ...
%                 * (previousTotal:(totalSamples - 1));
%             ylim(axes1,[(-1 * yRange) yRange]);
% 
%             plot(axes1, time, bufferChAmV); %, time, bufferChBmV);
            missed_phases = 0;
            
            peaks = find(bufferChAmV ==  500);
            peak = peaks(length(peaks));
            
            bufferChAmV = bufferChAmV((peak + 312):(peak + 3000));
%             bufferChAmV(abs(bufferChAmV) < 12) = 0;
            
            pulse_peaks = find(bufferChAmV > 12);
            a = size(pulse_peaks);
            if (a(1) ~= 0)
                if (length(pulse_peaks) == 1)
                    first_peak = pulse_peaks(1);
                else
                    bins = discretize(pulse_peaks, 2);
                    first_peak = pulse_peaks(bins == 1);
                end

                peak_vals = bufferChAmV(first_peak);

                [M,I] = max(peak_vals);

                fin_index = first_peak(I);

                time = fin_index * 3.2e-8;


                z_mot = (time - target_time) * 1500;

                z_motT = z_mot * scale_motion;

                % fclose(socket);

                msg_format = 'T%02dZ%09d\n';
                msg = sprintf(msg_format, [mod(ind,100), round(z_motT)]);

                write(socket, msg);

%                 zHist(ind, 1) = time * 1.5;
%                 if ind == 1
%                     zHist(ind, 2) = LoadTime;
%                 else
%                     zHist(ind, 2) = zHist(ind - 1, 2) + LoadTime;
%                 end
            end
%             zHist(ind, 1) = time * 1.5;
%             if ind == 1
%                 zHist(ind, 2) = LoadTime;
%             else
%                 zHist(ind, 2) = zHist(ind - 1, 2) + LoadTime;
%             end
            
%           pulse_peaks = pulse_peaks(3:(round(length(pulse_peaks) / 2)));
            
            tic
        end
       
        % Clear variables for use again
        clear bufferChAmV;
        clear firstValuePosn;
        clear lastValuePosn;
        clear startIndex;
        clear triggered;
        clear triggerAt;
        
   end
   
    % Check if auto stop has occurred.
    hasAutoStopOccurred = invoke(streamingGroupObj, 'autoStopped');

    if (hasAutoStopOccurred == PicoConstants.TRUE)

       disp('AutoStop: TRUE - exiting data collection loop.');
       break;

    end
   
    % Check if 'STOP' button has been clicked.
    flag = getappdata(gcf, 'run');
    drawnow;

    if (flag == 0)
        disp('STOP button clicked - aborting data collection.')
        break;
    end
 
end

% Close the STOP button window.
if (exist('stopFig', 'var'))
    
    close('Stop Button');
    clear stopFig;
        
end

if (hasTriggered == PicoConstants.TRUE)
   
    fprintf('Triggered at overall index: %d\n\n', triggeredAtIndex);
    
end

fprintf('\n');


%% Disconnect device
% Disconnect device object from hardware.
[status.stop] = invoke(ps5000aDeviceObj, 'ps5000aStop');

disconnect(ps5000aDeviceObj);
delete(ps5000aDeviceObj);


plot(zHist(:,2), zHist(:,1))
xlabel('time (s)')
ylabel('zDistance, (1mm)')
yline(target_time * 1.5)
yline((target_time * 1.5) + 0.5e-6)
yline((target_time * 1.5) - 0.5e-6)