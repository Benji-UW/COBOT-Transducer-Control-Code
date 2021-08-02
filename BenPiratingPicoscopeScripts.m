%% PicoScope 5000 Series (A API) Instrument Driver Oscilloscope Streaming Data Capture Example
% This is an example of an instrument control session using a device 
% object. The instrument control session comprises all the steps you 
% are likely to take when communicating with your instrument. 
%       
% These steps are:
%    
% # Create a device object   
% # Connect to the instrument 
% # Configure properties 
% # Invoke functions 
% # Disconnect from the instrument 
%
% To run the instrument control session, type the name of the file,
% PS5000A_ID_Streaming_Example, at the MATLAB command prompt.
% 
% The file, PS5000A_ID_STREAMING_EXAMPLE.M must be on your MATLAB PATH. For
% additional information on setting your MATLAB PATH, type 'help addpath'
% at the MATLAB command prompt.
%
% *Example:*
%     PS5000A_ID_Streaming_Example;
%
% *Description:*
%     Demonstrates how to set properties and call functions in order
%     to capture data in streaming mode from a PicoScope 5000 Series
%     Oscilloscope using the underlying 'A' API library functions.
%
% *Note:* Not all device functions used in this example are compatible with
% the Test and Measurement Tool.
%
% *See also:* <matlab:doc('icdevice') |icdevice|> | <matlab:doc('instrument/invoke') |invoke|>
%
% *Copyright:* © 2013-2018 Pico Technology Ltd. See LICENSE file for terms.

%% Suggested input test signals
% This example was published using the following test signals:
%
% * Channel A: 3 Vpp, 1 Hz sine wave
% * Channel B: 2 Vpp, 4 Hz square wave 

%% Clear command window and close any figures

clc;
clear all;
close all;

%% Load configuration information

PS5000aConfig;

%% Parameter definitions
% Define any parameters that might be required throughout the script.

channelA = ps5000aEnuminfo.enPS5000AChannel.PS5000A_CHANNEL_A;
% channelB = ps5000aEnuminfo.enPS5000AChannel.PS5000A_CHANNEL_B;

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
channelSettings(1).range = ps5000aEnuminfo.enPS5000ARange.PS5000A_2V;
channelSettings(1).analogueOffset = 0.0;

channelARangeMv = PicoConstants.SCOPE_INPUT_RANGES(channelSettings(1).range + 1);

% % Channel B
% channelSettings(2).enabled = PicoConstants.TRUE;
% channelSettings(2).coupling = ps5000aEnuminfo.enPS5000ACoupling.PS5000A_DC;
% channelSettings(2).range = ps5000aEnuminfo.enPS5000ARange.PS5000A_2V;
% channelSettings(2).analogueOffset = 0.0;

% Variables that will be required later
channelBRangeMv = PicoConstants.SCOPE_INPUT_RANGES(channelSettings(2).range + 1);


% The following code for blocking the C and D channels has been commented out
% because it is known that the picoscope we're using has only two channels
% if (ps5000aDeviceObj.channelCount == PicoConstants.QUAD_SCOPE)

%     % Channel C
%     channelSettings(3).enabled = PicoConstants.FALSE;
%     channelSettings(3).coupling = ps5000aEnuminfo.enPS5000ACoupling.PS5000A_DC;
%     channelSettings(3).range = ps5000aEnuminfo.enPS5000ARange.PS5000A_2V;
%     channelSettings(3).analogueOffset = 0.0;

%     % Channel D
%     channelSettings(4).enabled = PicoConstants.FALSE;
%     channelSettings(4).coupling = ps5000aEnuminfo.enPS5000ACoupling.PS5000A_DC;
%     channelSettings(4).range = ps5000aEnuminfo.enPS5000ARange.PS5000A_2V;
%     channelSettings(4).analogueOffset = 0.0;
    
% end

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

% Set resolution to 15 bits as 2 channels will be enabled.
[status.setResolution, resolution] = invoke(ps5000aDeviceObj, 'ps5000aSetDeviceResolution', 15);  

% Obtain the maximum Analog Digital Converter (ADC) count value from the
% driver - this is used for scaling values returned from the driver when
% data is collected. This value may change depending on the resolution
% selected.
maxADCCount = get(ps5000aDeviceObj, 'maxADCValue');

%% Set simple trigger
% Set a trigger on channel A, with an auto timeout - the default value for
% delay is used. The device will wait for a rising edge through
% the specified threshold unless the timeout occurs first.

% Trigger properties and functions are located in the Instrument
% Driver's Trigger group.

triggerGroupObj = get(ps5000aDeviceObj, 'Trigger');
triggerGroupObj = triggerGroupObj(1);

% Set the |autoTriggerMs| property in order to automatically trigger the
% oscilloscope after 1 second if a trigger event has not occurred. Set to 0
% to wait indefinitely for a trigger event.

set(triggerGroupObj, 'autoTriggerMs', 0);

% Channel     : 0 (ps5000aEnuminfo.enPS5000AChannel.PS5000A_CHANNEL_A)
% Threshold   : 500 mV
% Direction   : 2 (ps5000aEnuminfo.enPS5000AThresholdDirection.PS5000A_RISING)

[status.setSimpleTrigger] = invoke(triggerGroupObj, 'setSimpleTrigger', 0, 100, 2);


%% Set data buffers
% Data buffers for channels A and B - buffers should be set with the driver,
% and these *MUST* be passed with application buffers to the wrapper driver.
% This will ensure that data is correctly copied from the driver buffers
% for later processing.

overviewBufferSize  = 100000; % Size of the buffer to collect data from buffer.
segmentIndex        = 0;   
ratioMode           = ps5000aEnuminfo.enPS5000ARatioMode.PS5000A_RATIO_MODE_NONE;

% Buffers to be passed to the driver
pDriverBufferChA = libpointer('int16Ptr', zeros(overviewBufferSize, 1, 'int16'));
% Removed due to it being Channel B
% pDriverBufferChB = libpointer('int16Ptr', zeros(overviewBufferSize, 1, 'int16'));

status.setDataBufferChA = invoke(ps5000aDeviceObj, 'ps5000aSetDataBuffer', ...
    channelA, pDriverBufferChA, overviewBufferSize, segmentIndex, ratioMode);

% status.setDataBufferChB = invoke(ps5000aDeviceObj, 'ps5000aSetDataBuffer', ...
%     channelB, pDriverBufferChB, overviewBufferSize, segmentIndex, ratioMode);

% Application Buffers - these are for copying from the driver into.
pAppBufferChA = libpointer('int16Ptr', zeros(overviewBufferSize, 1, 'int16'));
% pAppBufferChB = libpointer('int16Ptr', zeros(overviewBufferSize, 1, 'int16'));

% Streaming properties and functions are located in the Instrument Driver's
% Streaming group.

streamingGroupObj = get(ps5000aDeviceObj, 'Streaming');
streamingGroupObj = streamingGroupObj(1);

status.setAppDriverBuffersA = invoke(streamingGroupObj, 'setAppAndDriverBuffers', channelA, ...
    pAppBufferChA, pDriverBufferChA, overviewBufferSize);

% status.setAppDriverBuffersB = invoke(streamingGroupObj, 'setAppAndDriverBuffers', channelB, ...
%     pAppBufferChB, pDriverBufferChB, overviewBufferSize);




%% Disconnect device
% Disconnect device object from hardware.

disconnect(ps5000aDeviceObj);
delete(ps5000aDeviceObj);