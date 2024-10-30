function agent = createLatAgentPinWang(observationInfo,actionInfo,Ts)

statePath = [
    featureInputLayer(observationInfo.Dimension(1),'Normalization','zscore','Name','observation')
    
    fullyConnectedLayer(300, 'Name', 'fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(300, 'Name', 'fc2')
    additionLayer(2,'Name','add')
    quadraticLayer
    fullyConnectedLayer(1,'Name','fc5')
    ];

actionPath = [
    featureInputLayer(actionInfo.Dimension(1),'Normalization','zscore','Name','action')
    fullyConnectedLayer(300, 'Name', 'fc6')
    ];

criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork, actionPath);
criticNetwork = connectLayers(criticNetwork,'fc6','add/in2');
criticNetwork = dlnetwork(criticNetwork);


criticOptions = rlOptimizerOptions('LearnRate',1e-3,'GradientThreshold',1);


critic = rlQValueFunction(criticNetwork,observationInfo,actionInfo,...
    'ObservationInputNames','observation','ActionInputNames','action');


actorNetwork = [
    featureInputLayer(observationInfo.Dimension(1),'Normalization','zscore','Name','observation')
    fullyConnectedLayer(300, 'Name', 'fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(300, 'Name', 'fc2')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(1, 'Name', 'fc3')
    reluLayer('Name', 'relu3')
    ];
actorNetwork = dlnetwork(actorNetwork);

actorOptions = rlOptimizerOptions('LearnRate',1e-4,'GradientThreshold',1);
actor = rlContinuousDeterministicActor(actorNetwork,observationInfo,actionInfo);


agentOptions = rlDDPGAgentOptions(...
    'SampleTime', Ts,...
    'CriticOptimizerOptions', criticOptions,...  % Assuming criticOptions are adjusted as previously discussed
    'ActorOptimizerOptions', actorOptions,...
    'ExperienceBufferLength', 1e4,...
    'DiscountFactor', 0.99); % Slightly more emphasis on future rewards


agentOptions.NoiseOptions.Variance = 0.3;  % Initial variance
agentOptions.NoiseOptions.VarianceDecayRate = 1e-5;  % Slower decay to encourage prolonged exploration

% agentOptions.TargetUpdateMethod = 'smoothing';
% agentOptions.TargetSmoothFactor = 1e-4;  % Smoother updates to target network


agent = rlDDPGAgent(actor,critic,agentOptions);


end
