load Maps simpleMap
mapMatrix = simpleMap;
mapScale = 1;
scanAngles = [-3*pi/8 : pi/8 :3*pi/8];
maxRange = 12;
lidarNoiseVariance = 0.1^2;
lidarNoiseSeeds = randi(intmax,size(scanAngles));
% Max speed parameters
maxLinSpeed = 0.3;
maxAngSpeed = 0.3;

% Initial pose of the robot
initX1 = 5;
initY1 = 22;
initX2 = 22;
initY2 = 22;
initTheta1 = -pi/2;
initTheta2 = -pi/2;

fig = figure("Name","simpleMap");
set(fig, "Visible", "on");
ax = axes(fig);

show(binaryOccupancyMap(mapMatrix),"Parent",ax);
hold on
plotTransforms([initX1, initY1, 0], eul2quat([initTheta1, 0, 0]), "MeshFilePath","groundvehicle.stl", "View", "2D");
plotTransforms([initX2, initY2, 0], eul2quat([initTheta2, 0, 0]), "MeshFilePath","groundvehicle.stl", "View", "2D");
light;
hold off

mdl = "MultiMobileRobotObstacleAvoidance";

Tfinal = 100;
sampleTime = 0.1;

agentBlk = [mdl + "/Agent1", mdl + "/Agent2"];
open_system(mdl)
%%
obsInfo = rlNumericSpec([numel(scanAngles) 1],...
    "LowerLimit",zeros(numel(scanAngles),1),...
    "UpperLimit",ones(numel(scanAngles),1)*maxRange);
numObservations = obsInfo.Dimension(1);

numActions = 2;
actInfo = rlNumericSpec([numActions 1],...
    "LowerLimit",-1,...
    "UpperLimit",1);

obsInfos = {obsInfo, obsInfo};
actInfos = {actInfo, actInfo};
env = rlSimulinkEnv(mdl,agentBlk,obsInfos,actInfos);
env.ResetFcn = @(in)MultiRLAvoidObstaclesResetRandFcn(in,scanAngles,maxRange,mapMatrix);

env.UseFastRestart = "Off";
%%
statePath = [
    imageInputLayer([numObservations 1 1], "Normalization", "none", "Name", "State")
    fullyConnectedLayer(50, "Name", "CriticStateFC1")
    reluLayer("Name", "CriticRelu1")
    fullyConnectedLayer(25, "Name", "CriticStateFC2")];
actionPath = [
    imageInputLayer([numActions 1 1], "Normalization", "none", "Name", "Action")
    fullyConnectedLayer(25, "Name", "CriticActionFC1")];
commonPath = [
    additionLayer(2,"Name", "add")
    reluLayer("Name","CriticCommonRelu")
    fullyConnectedLayer(1, "Name", "CriticOutput")];

criticNetwork = layerGraph();
criticNetwork = addLayers(criticNetwork,statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
criticNetwork = addLayers(criticNetwork,commonPath);
criticNetwork = connectLayers(criticNetwork,"CriticStateFC2","add/in1");
criticNetwork = connectLayers(criticNetwork,"CriticActionFC1","add/in2");

criticOpts = rlRepresentationOptions("LearnRate",1e-3,"L2RegularizationFactor",1e-4,"GradientThreshold",1);
critic1 = rlQValueRepresentation(criticNetwork,obsInfo,actInfo,"Observation",{'State'},"Action",{'Action'},criticOpts);
critic2 = rlQValueRepresentation(criticNetwork,obsInfo,actInfo,"Observation",{'State'},"Action",{'Action'},criticOpts);

actorNetwork = [
    imageInputLayer([numObservations 1 1], "Normalization", "none", "Name", "State")
    fullyConnectedLayer(50, "Name", "actorFC1")
    reluLayer("Name","actorReLU1")
    fullyConnectedLayer(50, "Name", "actorFC2")
    reluLayer("Name","actorReLU2")
    fullyConnectedLayer(2, "Name", "actorFC3")
    tanhLayer("Name", "Action")];
actorGraph = layerGraph(actorNetwork);

actorOptions = rlRepresentationOptions("LearnRate",1e-4,"L2RegularizationFactor",1e-4,"GradientThreshold",1);
actor1 = rlDeterministicActorRepresentation(actorNetwork,obsInfo,actInfo,"Observation",{'State'},"Action",{'Action'},actorOptions);
actor2 = rlDeterministicActorRepresentation(actorNetwork,obsInfo,actInfo,"Observation",{'State'},"Action",{'Action'},actorOptions);

%%

