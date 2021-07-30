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


%% Disconnect device
% Disconnect device object from hardware.

disconnect(ps5000aDeviceObj);
delete(ps5000aDeviceObj);