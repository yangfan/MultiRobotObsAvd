fig = figure("Name","simpleMap");
set(fig, "Visible", "on");
ax = axes(fig);
show(binaryOccupancyMap(mapMatrix),"Parent",ax);

for i = 1:5:size(out.range1, 3)
    u1 = out.pose1(i, :);
    r1 = out.range1(:, :, i);
    u2 = out.pose2(i, :);
    r2 = out.range2(:, :, i);
    u = [u1; u2];
    r = [r1, r2];
    PlotAvoidObstaclesPose(u, mapMatrix, mapScale, r, scanAngles, ax);
end

