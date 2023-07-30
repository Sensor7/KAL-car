%%
circuit = ones(24, 20);
for j=1:20
    for i=1:24
          circuit(i,j) = 0;
    end
end
res = 0.1; % meters
costmap = vehicleCostmap(circuit, 'cellSize', res );%或者不定义res，直接在'cellSize',后面输入分辨率
costmap.CollisionChecker.VehicleDimensions.Wheelbase = 0.3;
costmap.CollisionChecker.VehicleDimensions.RearOverhang = 0.02;
costmap.CollisionChecker.VehicleDimensions.Length = 0.35;
costmap.CollisionChecker.VehicleDimensions.Width = 0.18;
costmap.CollisionChecker.VehicleDimensions.Height = 0.10;
wheelbase = 0.3;
clear i j res circuit


refPoses = zeros(561, 3);
index = 1;
refPoses(1,:) = [1.8, 0.2, 90];
for i = 1:130
    refPoses(index+i,:) = [1.8, 0.2+0.01*i, 90];
end
index = index + 130;
for i = 1:150
    refPoses(index+i, :) = [1.4+0.4*cosd(i), 1.5+0.4*sind(i), 90+i];
end
index = index +150;
for i = 1:40
    refPoses(index+i, :) = [1.4-0.4*cosd(30)-0.01*cosd(60)*i, 1.7-0.01*sind(60)*i, 240];
end
index = index +40;
for i = 1:240
    refPoses(index+i, :) = [1.4-0.4*cosd(30)-0.4*cosd(60)-0.12^0.5+0.4*cosd(30+i), 0.2+1.7-0.4*sind(60)-0.4*sind(30+i), 240-i];
end
%index = index + 180;
%for i = 1:180
%    refPoses(index+1, :) = [1.2-0.2*cosd(30)-0.4*cosd(60)-0.03^0.5-0.2*sind(i), 0.1+1.6-0.4*sind(60)-0.2*cosd(i), 180-i];
%end
%index = index + 30;
startPose = refPoses(1,:);
clear i j index
%%
 curvatures = zeros(561,1);
 directions = ones(561,1);
 speed = 0.1;
 speedprofile = ones(561,1).*speed;
 speedprofile(end) = 0.0;
 
 %%
 [refPoses,directions,~,curvatures] = smoothPathSpline(refPoses,directions,561);
