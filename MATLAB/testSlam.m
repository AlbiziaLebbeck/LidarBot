% load('offlineSlamData.mat');
load('mapdata.mat');

scans = cell(1,length(data));
for i=1:length(data)
    scans{1,i} = lidarScan(data{1,i}.Ranges,data{1,i}.Angles);
end


maxLidarRange = 8;
mapResolution = 20;
slamAlg = robotics.LidarSLAM(mapResolution, maxLidarRange);

slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

firstTimeLCDetected = false;

figure;
for i=1:3:length(scans)
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{i});
    if ~isScanAccepted
        continue;
    end
    fprintf('Added scan %d \n', i);
    % visualize the first detected loop closure, if you want to see the
    % complete map building process, remove the if condition below
%     if optimizationInfo.IsPerformed && ~firstTimeLCDetected
        show(slamAlg, 'Poses', 'off');
        hold on;
        show(slamAlg.PoseGraph); 
        hold off;
        firstTimeLCDetected = true;
        drawnow
%     end
end
title('First loop closure');