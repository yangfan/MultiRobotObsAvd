function PlotAvoidObstaclesPose(u, mapMatrix, mapScale, range, scanAngles, ax)
%POSEPLOT Summary of this function goes here
%   Detailed explanation goes here

cla(ax);
show(binaryOccupancyMap(mapMatrix, mapScale), "Parent", ax);
hold(ax, "on");
ang = zeros(size(u,1),1);
plotTransforms([u(:,1), u(:,2), ang], eul2quat([u(:,3), ang, ang]), "MeshFilePath", "groundvehicle.stl", "View", "2D", "Parent", ax);
scan1 = lidarScan(range(:,1), scanAngles);
scan1 = transformScan(scan1, u(1,:));
scan2 = lidarScan(range(:,2), scanAngles);
scan2 = transformScan(scan2, u(2,:));
scanX = [scan1.Cartesian(:, 1); scan2.Cartesian(:, 1)];
scanY = [scan1.Cartesian(:, 2); scan2.Cartesian(:, 2)];
plot(ax, scanX, scanY, 'rX');
hold(ax, "off");
drawnow;