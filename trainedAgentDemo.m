% createEnvMulti;
% load('demo\agent148.mat');
obstacleAvoidanceAgent1 = saved_agent(1);
obstacleAvoidanceAgent2 = saved_agent(2);
out= sim('MultiMobileRobotObstacleAvoidance.slx');
showResults