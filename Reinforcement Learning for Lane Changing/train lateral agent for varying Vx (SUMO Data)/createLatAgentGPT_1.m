function agent = createLatAgentGPT_1(observationInfo,actionInfo,Ts)

statePath = [
    featureInputLayer(observationInfo.Dimension(1),'Normalization','zscore','Name','observation')
    fullyConnectedLayer(400, 'Name', 'fc1', 'WeightsInitializer', 'he', 'BiasInitializer', 'zeros', 'L2Factor', 0.01)
    batchNormalizationLayer('Name','bn1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(300, 'Name', 'fc2', 'WeightsInitializer', 'he', 'BiasInitializer', 'zeros', 'L2Factor', 0.01)
    batchNormalizationLayer('Name','bn2')
    additionLayer(2,'Name','add')
    reluLayer('Name','relu4')
    fullyConnectedLayer(1,'Name','fc5')
    ];

actionPath = [
    featureInputLayer(actionInfo.Dimension(1),'Normalization','zscore','Name','action')
    fullyConnectedLayer(300, 'Name', 'fc6', 'WeightsInitializer', 'he', 'BiasInitializer', 'zeros', 'L2Factor', 0.01)
    ];

criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork, actionPath);
criticNetwork = connectLayers(criticNetwork,'fc6','add/in2');
criticNetwork = dlnetwork(criticNetwork);


criticOptions = rlOptimizerOptions('LearnRate',1e-4,'GradientThreshold',1);


critic = rlQValueFunction(criticNetwork,observationInfo,actionInfo,...
    'ObservationInputNames','observation','ActionInputNames','action');


actorNetwork = [
    featureInputLayer(observationInfo.Dimension(1),'Normalization','zscore','Name','observation')
    fullyConnectedLayer(400, 'Name', 'fc1', 'WeightsInitializer', 'he', 'BiasInitializer', 'zeros', 'L2Factor', 0.01)
    batchNormalizationLayer('Name','bn1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(300, 'Name', 'fc2', 'WeightsInitializer', 'he', 'BiasInitializer', 'zeros', 'L2Factor', 0.01)
    reluLayer('Name', 'relu2')
    batchNormalizationLayer('Name','bn2')
    reluLayer('Name','relu2')
    fullyConnectedLayer(1, 'Name', 'fc3', 'WeightsInitializer', 'glorot', 'BiasInitializer', 'zeros')
    batchNormalizationLayer('Name','bn3')
    tanhLayer('Name', 'tanh1')
    ];
actorNetwork = dlnetwork(actorNetwork);

actorOptions = rlOptimizerOptions('LearnRate',5e-4,'GradientThreshold',1);
actor = rlContinuousDeterministicActor(actorNetwork,observationInfo,actionInfo);


agentOptions = rlDDPGAgentOptions(...
    'SampleTime', Ts,...
    'CriticOptimizerOptions', criticOptions,...  % Assuming criticOptions are adjusted as previously discussed
    'ActorOptimizerOptions', actorOptions,...
    'ExperienceBufferLength', 2e4,...
    'MiniBatchSize', 128, ... % Experiment with different sizes
    'DiscountFactor', 0.99); % Slightly more emphasis on future rewards


agentOptions.NoiseOptions.Variance = 0.3;  % Initial variance
agentOptions.NoiseOptions.VarianceDecayRate = 1e-4;  % Slower decay to encourage prolonged exploration

agentOptions.TargetUpdateMethod = 'smoothing';
agentOptions.TargetSmoothFactor = 1e-4;  % Smoother updates to target network


agent = rlDDPGAgent(actor,critic,agentOptions);


end










