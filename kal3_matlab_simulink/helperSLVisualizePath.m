function helperSLVisualizePath(pose, steer, refPoses, costmapStruct, vehicleDimsStruct)

%helperSLVisualizePath visualize reference path and vehicle position.

% Copyright 2017-2018 The MathWorks, Inc.

persistent pathPoints vehicleDims costmap vehicleBodyHandle 

if isempty(costmap)
    % Initialize vehicle dimensions object
    vehicleDims = vehicleDimensions( ...
        vehicleDimsStruct.Length, ...
        vehicleDimsStruct.Width, ...
        vehicleDimsStruct.Height, ...
        'Wheelbase',        vehicleDimsStruct.Wheelbase, ...
        'RearOverhang',     vehicleDimsStruct.RearOverhang, ...
        'WorldUnits',       char(vehicleDimsStruct.WorldUnits));
    
    ccConfig = inflationCollisionChecker(vehicleDims, 4);
    
    % Initialize vehicleCostmap object
    costmap = vehicleCostmap(costmapStruct.Costs, ...
        'FreeThreshold',     costmapStruct.FreeThreshold, ...
        'OccupiedThreshold', costmapStruct.OccupiedThreshold, ...
        'MapLocation',       costmapStruct.MapExtent([1, 3]), ...
        'CellSize',          costmapStruct.CellSize, ...
        'CollisionChecker',  ccConfig);
    
    % Create figure window if none already exists
    
    
end

if isempty(pathPoints)
    pathPoints = zeros(size(refPoses, 1), 2);
end

% Plot smooth path and map
if ~isequal(pathPoints, refPoses(:,1:2))
    % Initialize figure
    if ~any(pathPoints)
        % Plot path
        close(findobj('Type', 'Figure', 'Name', 'Automated Parking Valet'));
        fh             = figure;
        fh.Name        = 'Automated Parking Valet';
        fh.NumberTitle = 'off';
        ax             = axes(fh);
        
        plot(costmap, 'Parent', ax, 'Inflation', 'off');
        legend off
        
        hold(ax, 'on');
        title(ax, '');
        
        ax.XLim = costmap.MapExtent(1:2);
        ax.YLim = costmap.MapExtent(3:4);
    end
    
    % Plot smooth path
    plot(refPoses(:,1), refPoses(:,2),'b', 'LineWidth', 2);
    
    % Update path points for the new path segment
    pathPoints = refPoses(:,1:2);
end

% Plot trajectory
plot(pose(1), pose(2), 'r.');

% Plot vehicle
if isempty(vehicleBodyHandle) 
    vehicleBodyHandle = helperPlotVehicle(pose, vehicleDims, steer);
else
    vehicleShapes = helperVehiclePolyshape(pose, vehicleDims, steer);    
    for n = 1 : numel(vehicleBodyHandle)
        vehicleBodyHandle(n).Shape = vehicleShapes(n);
    end
end

uistack(vehicleBodyHandle, 'top');

drawnow('limitrate');

end


