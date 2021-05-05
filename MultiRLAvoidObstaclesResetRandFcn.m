function in = MultiRLAvoidObstaclesResetRandFcn(in,~,maxRange,mapMatrix)
% Reset function for reinforcement learning based obstacle avoidance with
% Multiple robots

    % Load map and lidar sensor (to generate valid pose)
    persistent map lidar
    if isempty(map) && isempty(lidar)
        map = binaryOccupancyMap(mapMatrix);
        lidar = rangeSensor('HorizontalAngle', [-3*pi/8, 3*pi/8], 'HorizontalAngleResolution', pi/8, 'RangeNoise', 0.01, 'Range', [0 maxRange]);
    end
    
 %% random initial configuration of robots   
    % Randomly generate pose inside the map. 
    % If the pose is in an unoccupied space and there are no range readings
    % nearby, assign this pose to the new simulation run
%     pos1Found = false;   
%     while(~pos1Found)
%         pos1 = [diff(map.XWorldLimits)*rand + map.XWorldLimits(1); ...
%             diff(map.YWorldLimits)*rand + map.YWorldLimits(1); ...
%             2*pi*rand];
%         ranges = lidar(pos1', map);
%         if ~checkOccupancy(map,pos1(1:2)') && all(ranges(~isnan(ranges)) >= 0.5)
%             pos1Found = true;
%             in = setVariable(in,'initX1', pos1(1));
%             in = setVariable(in,'initY1', pos1(2));
%             in = setVariable(in,'initTheta1', pos1(3));
%         end
%     end
    
%     pose_grid = world2grid(map, [pos1(1),pos1(2)]);
%     mapMatrix2 = mapMatrix;
%     mapMatrix2(pose_grid(1), pose_grid(2)) = true;
%     map2 = binaryOccupancyMap(mapMatrix2);
%     pos2Found = false;   
%     while(~pos2Found)
%         pos2 = [diff(map2.XWorldLimits)*rand + map2.XWorldLimits(1); ...
%                diff(map2.YWorldLimits)*rand + map2.YWorldLimits(1); ... 
%                2*pi*rand];  
%         ranges2 = lidar(pos2', map2);
%         if ~checkOccupancy(map2,pos2(1:2)') && all(ranges2(~isnan(ranges2)) >= 0.5)
%             pos2Found = true;
%             in = setVariable(in,'initX2', pos2(1));
%             in = setVariable(in,'initY2', pos2(2));
%             in = setVariable(in,'initTheta2', pos2(3));
%         end
%     end

%% fix the initial configuration of robots
    in = setVariable(in,'initX1', 5);
    in = setVariable(in,'initY1', 22);
    in = setVariable(in,'initX2', 22);
    in = setVariable(in,'initY2', 22);
    in = setVariable(in,'lidarNoiseSeeds',randi(intmax, lidar.NumReadings, 1)); % random seed of random sensor noise

end

