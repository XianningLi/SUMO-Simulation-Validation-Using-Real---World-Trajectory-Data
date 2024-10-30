clear all
close all
clc

% load model parametrs
rlAgentLCParams

% set the samnpling time
Ts = 0.1;
%  set the simulation time
Tf = 15; % simulation time

% create the timeseries velocity vector
load('subjectOrg43.mat')
timeVec = Ts:Ts:Tf;
timeVecOrg = Ts:Ts:60-Ts; % the NGSIM data is for 59 sec
vSub = subjectOrg43(:,12);
indx = ceil(linspace(1,length(vSub),Tf/Ts));
vSubRed = vSub(indx);
plot(timeVecOrg, vSub)
hold on
plot(timeVecOrg(indx), vSubRed, 'O')

vSubTimeSeries = timeseries(vSubRed, timeVec);

% Load or create the environment
mdl = 'rlAgentLCLatVaryingVx';
load_system(mdl);

% Create the observation info and action info
obsInfo = rlNumericSpec([4 1]);  % Assuming 3 continuous observations
actInfo = rlFiniteSetSpec((-10:0.01:10));

% Create the environment interface
blks = mdl + ["/RL Agent1"];
env = rlSimulinkEnv(mdl,blks,obsInfo,actInfo);


L = 24; % number of neurons
statePath = [
    featureInputLayer(obsInfo.Dimension(1),'Normalization','none','Name','observation')
    fullyConnectedLayer(L,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(L,'Name','fc2')
    additionLayer(2,'Name','add')
    reluLayer('Name','relu2')
    fullyConnectedLayer(L,'Name','fc3')
    reluLayer('Name','relu3')
    fullyConnectedLayer(1,'Name','fc4')];

actionPath = [
    featureInputLayer(actInfo.Dimension(1),'Normalization','none','Name','action')
    fullyConnectedLayer(L,'Name','fc5')];

criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork, actionPath);    
criticNetwork = connectLayers(criticNetwork,'fc5','add/in2');
criticNetwork = dlnetwork(criticNetwork);

criticOptions = rlOptimizerOptions('LearnRate',1e-3,'GradientThreshold',1);

critic = rlQValueFunction(criticNetwork,obsInfo,actInfo,...
    'ObservationInputNames','observation','ActionInputNames','action');


agentOptions = rlDQNAgentOptions(...
    'SampleTime',Ts,...
    'UseDoubleDQN',true,...
    'CriticOptimizerOptions',criticOptions,...
    'ExperienceBufferLength',1e6);

agent = rlDQNAgent(critic,agentOptions);

% Train the agent
maxEpisodes = 2000;
maxSteps = ceil(Tf/Ts);
trainingOptions = rlTrainingOptions(...
    'MaxEpisodes',maxEpisodes,...
    'MaxStepsPerEpisode',maxSteps,...
    'ScoreAveragingWindowLength',20,...
    'Verbose',true,...
    'Plots','training-progress',...
    'StopTrainingValue', 1);
trainResults = train(agent, env, trainingOptions);
