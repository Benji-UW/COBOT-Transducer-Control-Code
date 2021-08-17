%% Clear command window and close any figures

clc;
clear all;
close all;

%% Load configuration information

PS5000aConfig;

%% Parameter definitions
% Define any parameters that might be required throughout the script.

channelA = ps5000aEnuminfo.enPS5000AChannel.PS5000A_CHANNEL_A;

%% Device connection

% Check if an Instrument session using the device object |ps5000aDeviceObj|
% is still open, and if so, disconnect if the User chooses 'Yes' when prompted.
if (exist('ps5000aDeviceObj', 'var') && ps5000aDeviceObj.isvalid && strcmp(ps5000aDeviceObj.status, 'open'))
    
    openDevice = questionDialog(['Device object ps5000aDeviceObj has an open connection. ' ...
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

% Create a device object. 
ps5000aDeviceObj = icdevice('picotech_ps5000a_generic', ''); 

% Connect device object to hardware.
connect(ps5000aDeviceObj);

%% Display unit information

[status.getUnitInfo, unitInfo] = invoke(ps5000aDeviceObj, 'getUnitInfo');
disp(unitInfo);

%% Channel setup
% All channels are enabled by default - if the device is a 4-channel scope,
% switch off channels C and D so device can be set to 15-bit resolution.

% Ben here - We only use channel A for our readings with the transducer, so 
% I commented out the code I needed for that :)


% Channel A
channelSettings(1).enabled = PicoConstants.TRUE;
channelSettings(1).coupling = ps5000aEnuminfo.enPS5000ACoupling.PS5000A_DC;
channelSettings(1).range = ps5000aEnuminfo.enPS5000ARange.PS5000A_500MV;
channelSettings(1).analogueOffset = 0.0;

channelARangeMv = PicoConstants.SCOPE_INPUT_RANGES(channelSettings(1).range + 1);


% Keep the status values returned from the driver.
numChannels = get(ps5000aDeviceObj, 'channelCount');
status.setChannelStatus = zeros(numChannels, 1);

[status.currentPowerSource] = invoke(ps5000aDeviceObj, 'ps5000aCurrentPowerSource');

% % Check if the power supply is connected - channels C and D will not be
% % enabled on a 4-channel oscilloscope if it is only USB powered.
% if (status.currentPowerSource == PicoStatus.PICO_POWER_SUPPLY_NOT_CONNECTED)
    
%     numChannels = PicoConstants.DUAL_SCOPE;
    
% end

% Manually set the numChannels to Single_Scope because of the known setup.
numChannels = PicoConstants.SINGLE_SCOPE;


for ch = 1:numChannels
   
    status.setChannelStatus(ch) = invoke(ps5000aDeviceObj, 'ps5000aSetChannel', ...
        (ch - 1), channelSettings(ch).enabled, ...
        channelSettings(ch).coupling, channelSettings(ch).range, ...
        channelSettings(ch).analogueOffset);
    
end

%% Change resolution
% The maximum resolution will depend on the number of channels enabled.

% % Set resolution to 15 bits as 2 channels will be enabled.
% Originally this was set to 15 bits but I set it to 8 because that turns
% out to be satisfactory for my purposes
[status.setResolution, resolution] = invoke(ps5000aDeviceObj, 'ps5000aSetDeviceResolution', 8);  

% Obtain the maximum Analog Digital Converter (ADC) count value from the
% driver - this is used for scaling values returned from the driver when
% data is collected. This value may change depending on the resolution
% selected.
maxADCCount = get(ps5000aDeviceObj, 'maxADCValue');

%% Set Advanced trigger

triggerGroupObj = get(ps5000aDeviceObj, 'Trigger');
triggerGroupObj = triggerGroupObj(1);

% ChATriggerConditions.source = ps5000aEnuminfo.enPS5000AChannel.PS5000A_EXTERNAL;
% ChATriggerConditions.condition = ps5000aEnuminfo.enPS5000ATriggerState.PS5000A_CONDITION_TRUE;
% 
% % Clear any pre-existing trigger configurations that may have been set.
% info = ps5000aEnuminfo.enPS5000AConditionsInfo.PS5000A_CLEAR + ...
%     ps5000aEnuminfo.enPS5000AConditionsInfo.PS5000A_ADD;
% 
% % Set the condition for channel A
% [status.ps5000aSetTriggerChannelConditionsV2ChA] = invoke(triggerGroupObj, ...
%     'ps5000aSetTriggerChannelConditionsV2', ChATriggerConditions, info);
% 
% % *Trigger directions*
% %
% % Set the direction on which to trigger for each channel.
% %
% % Create an array of MATLAB structures corresponding to the
% % |tPS5000ADirection| structure in |ps5000aStructs|. Each structure in the
% % array defines the direction on which to trigger and also if it is a level
% % (edge) or window trigger.
% 
% TriggerDirections(1).source = ps5000aEnuminfo.enPS5000AChannel.PS5000A_EXTERNAL;
% TriggerDirections(1).direction = ps5000aEnuminfo.enPS5000AThresholdDirection.PS5000A_RISING;
% TriggerDirections(1).mode = ps5000aEnuminfo.enPS5000AThresholdMode.PS5000A_LEVEL;
% 
% [status.setTriggerChannelDirectionsV2] = invoke(triggerGroupObj, ...
%     'ps5000aSetTriggerChannelDirectionsV2', TriggerDirections);

set(triggerGroupObj, 'autoTriggerMs', 100);

% Channel     : 0 (ps5000aEnuminfo.enPS5000AChannel.PS5000A_CHANNEL_A)
% Threshold   : 500 mV
% Direction   : 2 (ps5000aEnuminfo.enPS5000AThresholdDirection.PS5000A_RISING)

channel = ps5000aEnuminfo.enPS5000AChannel.PS5000A_EXTERNAL;

[status.setSimpleTrigger] = invoke(triggerGroupObj, 'setSimpleTrigger', 0, 250, 2);

%% Set data buffers
% Data buffers for channels A and B - buffers should be set with the driver,
% and these *MUST* be passed with application buffers to the wrapper driver.
% This will ensure that data is correctly copied from the driver buffers
% for later processing.

overviewBufferSize  = (312500 * 1.00); % Size of the buffer to collect data from buffer.
segmentIndex        = 0;
ratioMode           = ps5000aEnuminfo.enPS5000ARatioMode.PS5000A_RATIO_MODE_NONE;

% Buffers to be passed to the driver
pDriverBufferChA = libpointer('int16Ptr', zeros(overviewBufferSize, 1, 'int16'));

status.setDataBufferChA = invoke(ps5000aDeviceObj, 'ps5000aSetDataBuffer', ...
    channelA, pDriverBufferChA, overviewBufferSize, segmentIndex, ratioMode);


% Application Buffers - these are for copying from the driver into.
pAppBufferChA = libpointer('int16Ptr', zeros(overviewBufferSize, 1, 'int16'));

% Streaming properties and functions are located in the Instrument Driver's
% Streaming group.

streamingGroupObj = get(ps5000aDeviceObj, 'Streaming');
streamingGroupObj = streamingGroupObj(1);

status.setAppDriverBuffersA = invoke(streamingGroupObj, 'setAppAndDriverBuffers', channelA, ...
    pAppBufferChA, pDriverBufferChA, overviewBufferSize);

% status.setAppDriverBuffersB = invoke(streamingGroupObj, 'setAppAndDriverBuffers', channelB, ...
%     pAppBufferChB, pDriverBufferChB, overviewBufferSize);

%% Start streaming and collect data
% Use default value for streaming interval which is 1e-6 for 1 MS/s.
% Collect data for 5 seconds with auto stop - maximum array size will depend
% on the PC's resources - type <matlab:doc('memory') |memory|> at the
% MATLAB command prompt for further information.
%
% To change the sample interval set the |streamingInterval| property of the
% Streaming group object. The call to the |ps5000aRunStreaming()| function
% will output the actual sampling interval used by the driver.

% To change the sample interval e.g 5 us for 200 kS/s
set(streamingGroupObj, 'streamingInterval', 3.2e-8);

%%
% Set the number of pre- and post-trigger samples.
% If no trigger is set the library will still store
% |numPreTriggerSamples| + |numPostTriggerSamples|.
set(ps5000aDeviceObj, 'numPreTriggerSamples', 0);
% I'm setting the number of post-trigger samples to be 10 kS, because by default
% the streaming interval is 1 MS/s and the frequency of the pulser/receiver is 
% 100 Hz, meaning we want weach sample to capture just one of those
set(ps5000aDeviceObj, 'numPostTriggerSamples', 31250); % 1e4

%%
% The |autoStop| parameter can be set to false (0) to allow for continuous
% data collection.
set(streamingGroupObj, 'autoStop', PicoConstants.FALSE);

% Set other streaming parameters
downSampleRatio = 1;
downSampleRatioMode = ps5000aEnuminfo.enPS5000ARatioMode.PS5000A_RATIO_MODE_NONE;

%%
% Defined buffers to store data collected from the channels. If capturing
% data without using the autoStop flag, or if using a trigger with the
% autoStop flag, allocate sufficient space (1.5 times the sum of the number
% of pre-trigger and post-trigger samples is shown below) to allow for
% additional pre-trigger data. Pre-allocating the array is more efficient
% than using <matlab:doc('vertcat') |vertcat|> to combine data.

maxSamples = get(ps5000aDeviceObj, 'numPreTriggerSamples') + ...
    get(ps5000aDeviceObj, 'numPostTriggerSamples');

% Take into account the downsampling ratio mode - required if collecting
% data without a trigger and using the autoStop flag.

finalBufferLength = round(1.5 * maxSamples / downSampleRatio);

pBufferChAFinal = libpointer('int16Ptr', zeros(finalBufferLength, 1, 'int16'));
% pBufferChBFinal = libpointer('int16Ptr', zeros(finalBufferLength, 1, 'int16'));

% Prompt User to indicate if they wish to plot live streaming data.
plotLiveData = questionDialog('Plot live streaming data?', 'Streaming Data Plot');

if (plotLiveData == PicoConstants.TRUE)
   
    disp('Live streaming data collection with second plot on completion.');
    
else
    
    disp('Streaming data plot on completion.');
    
end

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

if (plotLiveData == PicoConstants.TRUE)
    
    % Plot on a single figure. 
    figure1 = figure('Name','PicoScope 5000 Series (A API) Example - Streaming Mode Capture', ...
         'NumberTitle','off');
     
    axes1 = axes('Parent', figure1);

    % Estimate x-axis limit to try and avoid using too much CPU resources
    % when drawing - use max voltage range selected if plotting multiple
    % channels on the same graph.
    xlim(axes1, [0 (sampleInterval * finalBufferLength)]);

    yRange = max(channelARangeMv); %, channelBRangeMv);
    ylim(axes1,[(-1 * yRange) yRange]);

    % turned hold off because I only want to see one pulse of data collected
    hold(axes1, 'off');
    % hold(axes1,'on');
    grid(axes1, 'on');

    title(axes1, 'Live Streaming Data Capture');
    
    if (strcmp(sampleIntervalTimeUnitsStr, 'us'))
        
        xLabelStr = 'Time (\mus)';
        
    else
       
        xLabelStr = strcat('Time (', sampleIntervalTimeUnitsStr, ')');
        xlabel(axes1, xLabelStr);
        
    end
    
    xlabel(axes1, xLabelStr);
    ylabel(axes1, 'Voltage (mV)');
    
end

%%
% Collect samples as long as the |hasAutoStopOccurred| flag has not been
% set or the call to |getStreamingLatestValues()| does not return an error
% code (check for STOP button push inside loop).

avg_samp_freq = zeros(150, 1);
ind = 1;

missed_phases = 0;
tic
while(hasAutoStopOccurred == PicoConstants.FALSE && ...
        status.getStreamingLatestValuesStatus == PicoStatus.PICO_OK && ind < 151)
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
        
        if (plotLiveData == PicoConstants.TRUE && max(bufferChAmV) > 100)% && startIndex == 0)
            LoadTime = toc;
        
            fprintf('Samples recording peaks at %f Hz, skipped %d sample periods. \n', ...
                (1 / LoadTime), missed_phases);
            tic
            
            avg_samp_freq(ind) = 1 / LoadTime;
            ind = ind + 1;
            
            % Time axis. 
            % Multiply by ratio mode as samples get reduced
%             time = (double(sampleInterval) * double(downSampleRatio)) * (previousTotal:(totalSamples - 1));
%             ylim(axes1,[(-1 * yRange) yRange]);
% 
%             plot(axes1, time, bufferChAmV); %, time, bufferChBmV);
            missed_phases = 0;
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

if (plotLiveData == PicoConstants.TRUE)
    
    drawnow;
    
    % Take hold off the current figure.
    hold(axes1, 'off');
    movegui(figure1, 'west');
    
end

if (hasTriggered == PicoConstants.TRUE)
   
    fprintf('Triggered at overall index: %d\n\n', triggeredAtIndex);
    
end

fprintf('\n');

%% Stop the device
% This function should be called regardless of whether the autoStop
% property is enabled or not.

[status.stop] = invoke(ps5000aDeviceObj, 'ps5000aStop');

%% Find the number of samples
% This is the number of samples held in the shared library itself. The
% actual number of samples collected when using a trigger is likely to be
% greater.

[status.noOfStreamingValues, numStreamingValues] = invoke(streamingGroupObj, 'ps5000aNoOfStreamingValues');

fprintf('Number of samples available after data collection: %u\n', numStreamingValues);

%% Process data
% Process data post-capture if required - here the data will be plotted.

% Reduce size of arrays if required.

if (totalSamples < finalBufferLength)
    
    pBufferChAFinal.Value(totalSamples + 1:end) = [];
%     pBufferChBFinal.Value(totalSamples + 1:end) = [];

end

% Retrieve data for the channels.
channelAFinal = pBufferChAFinal.Value();
% channelBFinal = pBufferChBFinal.Value();

% Plot total data collected on another figure.

finalFigure = figure('Name','PicoScope 5000 Series (A API) Example - Streaming Mode Capture', ...
    'NumberTitle','off');

finalFigureAxes = axes('Parent', finalFigure);
hold(finalFigureAxes, 'on');
grid(finalFigureAxes, 'on');

if (strcmp(sampleIntervalTimeUnitsStr, 'us'))
        
    xlabel(finalFigureAxes, 'Time (\mus)');

else

    xLabelStr = strcat('Time (', sampleIntervalTimeUnitsStr, ')');
    xlabel(finalFigureAxes, xLabelStr);

end

ylabel(finalFigureAxes, 'Voltage (mV)');
hold(finalFigureAxes, 'off');

time = (double(sampleInterval) * double(downSampleRatio)) * (0:length(channelAFinal) - 1);

% Channel A
% plot(time, channelAFinal, 'b');
avg_samp_freq = nonzeros(avg_samp_freq);
plot(avg_samp_freq, 'b')
hold on
mov_avg = movmean(avg_samp_freq, 10);
plot(mov_avg, 'r');
xLabelStr = strcat('Time (', sampleIntervalTimeUnitsStr, ')');
xlabel(xLabelStr);
ylabel('Voltage (mV)');
title('Data acquisition on channel A (Final)');
grid('on');

movegui(finalFigure, 'east');

mean(avg_samp_freq)
%% Disconnect device
% Disconnect device object from hardware.

disconnect(ps5000aDeviceObj);
delete(ps5000aDeviceObj);