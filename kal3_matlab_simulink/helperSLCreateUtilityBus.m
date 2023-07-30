function helperSLCreateUtilityBus() 
% helperSLCreateUtilityBus define Simulink buses used in the model.

% Copyright 2017-2018 The MathWorks, Inc.

% Vehicle info 
elemsVehicleInfo(1)                 = Simulink.BusElement;
elemsVehicleInfo(1).Name            = 'CurrPose';
elemsVehicleInfo(1).Dimensions      = [1 3];
elemsVehicleInfo(1).DimensionsMode  = 'Fixed';
elemsVehicleInfo(1).DataType        = 'double';
elemsVehicleInfo(1).SampleTime      = -1;
elemsVehicleInfo(1).Complexity      = 'real';

elemsVehicleInfo(2)                 = Simulink.BusElement;
elemsVehicleInfo(2).Name            = 'CurrVelocity';
elemsVehicleInfo(2).Dimensions      = 1;
elemsVehicleInfo(2).DimensionsMode  = 'Fixed';
elemsVehicleInfo(2).DataType        = 'double';
elemsVehicleInfo(2).SampleTime      = -1;
elemsVehicleInfo(2).Complexity      = 'real';

elemsVehicleInfo(3)                 = Simulink.BusElement;
elemsVehicleInfo(3).Name            = 'CurrYawRate';
elemsVehicleInfo(3).Dimensions      = 1;
elemsVehicleInfo(3).DimensionsMode  = 'Fixed';
elemsVehicleInfo(3).DataType        = 'double';
elemsVehicleInfo(3).SampleTime      = -1;
elemsVehicleInfo(3).Complexity      = 'real';

elemsVehicleInfo(4)                 = Simulink.BusElement;
elemsVehicleInfo(4).Name            = 'CurrSteer';
elemsVehicleInfo(4).Dimensions      = 1;
elemsVehicleInfo(4).DimensionsMode  = 'Fixed';
elemsVehicleInfo(4).DataType        = 'double';
elemsVehicleInfo(4).SampleTime      = -1;
elemsVehicleInfo(4).Complexity      = 'real';

elemsVehicleInfo(5)                 = Simulink.BusElement;
elemsVehicleInfo(5).Name            = 'Direction';
elemsVehicleInfo(5).Dimensions      = 1;
elemsVehicleInfo(5).DimensionsMode  = 'Fixed';
elemsVehicleInfo(5).DataType        = 'double';
elemsVehicleInfo(5).SampleTime      = -1;
elemsVehicleInfo(5).Complexity      = 'real';

vehicleInfoBus                      = Simulink.Bus;
vehicleInfoBus.Elements             = elemsVehicleInfo;

clear elemsVehicleInfo;
assignin('base','vehicleInfoBus',   vehicleInfoBus);

