%%
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

ChATriggerConditions.source = ps5000aEnuminfo.enPS5000AChannel.PS5000A_EXTERNAL;
ChATriggerConditions.condition = ps5000aEnuminfo.enPS5000ATriggerState.PS5000A_CONDITION_TRUE;

info = ps5000aEnuminfo.enPS5000AConditionsInfo.PS5000A_CLEAR + ...
    ps5000aEnuminfo.enPS5000AConditionsInfo.PS5000A_ADD;

% Set the condition for channel A
[status.ps5000aSetTriggerChannelConditionsV2ChA] = invoke(triggerGroupObj, ...
    'ps5000aSetTriggerChannelConditionsV2', ChATriggerConditions, info);

TriggerDirections(1).source = ps5000aEnuminfo.enPS5000AChannel.PS5000A_EXTERNAL;
TriggerDirections(1).direction = ps5000aEnuminfo.enPS5000AThresholdDirection.PS5000A_RISING;
TriggerDirections(1).mode = ps5000aEnuminfo.enPS5000AThresholdMode.PS5000A_LEVEL;

[status.setTriggerChannelDirectionsV2] = invoke(triggerGroupObj, ...
    'ps5000aSetTriggerChannelDirectionsV2', TriggerDirections);


overviewBufferSize  = 250000; % Size of the buffer to collect data from buffer.
segmentIndex        = 0;   
ratioMode           = ps5000aEnuminfo.enPS5000ARatioMode.PS5000A_RATIO_MODE_NONE;







