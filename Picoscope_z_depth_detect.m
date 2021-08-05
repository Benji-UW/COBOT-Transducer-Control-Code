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
