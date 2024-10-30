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
actInfo = rlNumericSpec([1 1]);  % Assuming 1 continuous action

% Create the environment interface
blks = mdl + ["/RL Agent1"];
env = rlSimulinkEnv(mdl,blks,obsInfo,actInfo);


% Define the critic network architecture
criticLayerSizes = [400 300];
statePath = [
    featureInputLayer(obsInfo.Dimension(1), 'Normalization', 'rescale-zero-one', 'Name', 'state')
    fullyConnectedLayer(criticLayerSizes(1), 'Name', 'CriticStateFC1')
    reluLayer('Name', 'CriticRelu1')
    fullyConnectedLayer(criticLayerSizes(2), 'Name', 'CriticStateFC2')];
actionPath = [
    featureInputLayer(actInfo.Dimension(1), 'Normalization', 'rescale-zero-one', 'Name', 'action')
    fullyConnectedLayer(criticLayerSizes(2), 'Name', 'CriticActionFC1', 'BiasLearnRateFactor', 0)];
commonPath = [
    additionLayer(2,'Name', 'add')
    reluLayer('Name','CriticCommonRelu')
    fullyConnectedLayer(1, 'Name', 'CriticOutput')];

criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork, actionPath);
criticNetwork = addLayers(criticNetwork, commonPath);
criticNetwork = connectLayers(criticNetwork,'CriticStateFC2','add/in1');
criticNetwork = connectLayers(criticNetwork,'CriticActionFC1','add/in2');

% Define the actor network architecture
actorLayerSizes = [400 300];
actorNetwork = [
    featureInputLayer(obsInfo.Dimension(1), 'Normalization', 'rescale-zero-one', 'Name', 'state')
    fullyConnectedLayer(actorLayerSizes(1), 'Name', 'ActorFC1')
    reluLayer('Name', 'ActorRelu1')
    fullyConnectedLayer(actorLayerSizes(2), 'Name', 'ActorFC2')
    reluLayer('Name', 'ActorRelu2')
    fullyConnectedLayer(actInfo.Dimension(1), 'Name', 'ActorFC3')
    tanhLayer('Name', 'ActorTanh')
    ];

% Create the critic and actor representations
criticOptions = rlRepresentationOptions('Optimizer','adam','LearnRate',1e-03);
critic = rlQValueRepresentation(criticNetwork,obsInfo,actInfo,...
    'Observation',{'state'},'Action',{'action'},criticOptions);
actorOptions = rlRepresentationOptions('Optimizer','adam','LearnRate',1e-03);
actor = rlDeterministicActorRepresentation(actorNetwork,obsInfo,actInfo,...
    'Observation',{'state'},actorOptions);

% Set options for the agent
agentOptions = rlDDPGAgentOptions(...
    'SampleTime',Ts,...
    'TargetSmoothFactor',1e-3,...
    'ExperienceBufferLength',1e6,...
    'MiniBatchSize',64);
agent = rlDDPGAgent(actor, critic, agentOptions);

% agentOptions = rlDQNAgentOptions(...
%     'SampleTime',Ts,...
%     'UseDoubleDQN',true,...
%     'CriticOptimizerOptions',criticOptions,...
%     'ExperienceBufferLength',1e6);
% 
% agent = rlDQNAgent(critic,agentOptions);

% Train the agent
maxEpisodes = 5000;
maxSteps = ceil(Tf/Ts);
trainingOptions = rlTrainingOptions(...
    'MaxEpisodes',maxEpisodes,...
    'MaxStepsPerEpisode',maxSteps,...
    'ScoreAveragingWindowLength',20,...
    'Verbose',true,...
    'Plots','training-progress',...
    'StopTrainingValue', 1);
trainResults = train(agent, env, trainingOptions);
