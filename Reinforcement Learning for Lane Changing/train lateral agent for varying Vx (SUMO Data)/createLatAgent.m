function agent = createLatAgent(observationInfo,actionInfo,Ts)
L = 16; % number of neurons
statePath = [
    featureInputLayer(observationInfo.Dimension(1),'Normalization','none','Name','observation')
    fullyConnectedLayer(L,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(8,'Name','fc2')
    additionLayer(2,'Name','add')
    reluLayer('Name','relu3')
    fullyConnectedLayer(1,'Name','fc4')];

actionPath = [
    featureInputLayer(actionInfo.Dimension(1),'Normalization','none','Name','action')
    fullyConnectedLayer(8, 'Name', 'fc5')];

criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork, actionPath);
criticNetwork = connectLayers(criticNetwork,'fc5','add/in2');
criticNetwork = dlnetwork(criticNetwork);


criticOptions = rlOptimizerOptions('LearnRate',1e-3,'GradientThreshold',1);


critic = rlQValueFunction(criticNetwork,observationInfo,actionInfo,...
    'ObservationInputNames','observation','ActionInputNames','action');


actorNetwork = [
    featureInputLayer(observationInfo.Dimension(1),'Normalization','none','Name','observation')
    fullyConnectedLayer(4,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(3,'Name','fc2')
    reluLayer('Name','relu2')
    fullyConnectedLayer(1,'Name','fc4')
    tanhLayer('Name','tanh1')
    ];
actorNetwork = dlnetwork(actorNetwork);

actorOptions = rlOptimizerOptions('LearnRate',1e-3,'GradientThreshold',1);
actor = rlContinuousDeterministicActor(actorNetwork,observationInfo,actionInfo);


agentOptions = rlDDPGAgentOptions(...
    'SampleTime',Ts,...
    'CriticOptimizerOptions',criticOptions,...
    'ActorOptimizerOptions',actorOptions,...
    'ExperienceBufferLength',1e4,...
    'MiniBatchSize', 128, ...
    'NumStepsToLookAhead',3, ...
     'NumEpoch',2);
% agentOptions.NoiseOptions.Variance = 0.6;
% agentOptions.NoiseOptions.VarianceDecayRate = 1e-5;
opt = rlDDPGAgentOptions;
opt.NoiseOptions = rl.option.GaussianActionNoise;
opt.NoiseOptions.StandardDeviation = 0.05;


agent = rlDDPGAgent(actor,critic,agentOptions);
% agent = rlPPOAgent(actor,critic,agentOptions);










