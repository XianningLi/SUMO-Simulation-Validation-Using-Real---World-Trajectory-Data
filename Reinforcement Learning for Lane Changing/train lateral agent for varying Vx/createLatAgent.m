function agent = createLatAgent(observationInfo,actionInfo,Ts)
L = 48; % number of neurons
statePath = [
    featureInputLayer(observationInfo.Dimension(1),'Normalization','none','Name','observation')
    fullyConnectedLayer(L,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(L,'Name','fc2')
    additionLayer(2,'Name','add')
    reluLayer('Name','relu2')
    fullyConnectedLayer(L,'Name','fc3')
    reluLayer('Name','relu3')
    fullyConnectedLayer(1,'Name','fc4')];

actionPath = [
    featureInputLayer(actionInfo.Dimension(1),'Normalization','none','Name','action')
    fullyConnectedLayer(L, 'Name', 'fc5')];

criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork, actionPath);
criticNetwork = connectLayers(criticNetwork,'fc5','add/in2');
criticNetwork = dlnetwork(criticNetwork);


criticOptions = rlOptimizerOptions('LearnRate',1e-3,'GradientThreshold',1);


critic = rlQValueFunction(criticNetwork,observationInfo,actionInfo,...
    'ObservationInputNames','observation','ActionInputNames','action');


actorNetwork = [
    featureInputLayer(observationInfo.Dimension(1),'Normalization','none','Name','observation')
    fullyConnectedLayer(L,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(L,'Name','fc2')
    reluLayer('Name','relu2')
    fullyConnectedLayer(L,'Name','fc3')
    reluLayer('Name','relu3')
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
    'ExperienceBufferLength',1e6);
agentOptions.NoiseOptions.Variance = 0.6;
agentOptions.NoiseOptions.VarianceDecayRate = 1e-5;


agent = rlDDPGAgent(actor,critic,agentOptions);
% agent = rlPPOAgent(actor,critic,agentOptions);










