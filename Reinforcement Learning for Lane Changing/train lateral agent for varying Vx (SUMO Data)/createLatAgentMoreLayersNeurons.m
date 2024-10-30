function agent = createLatAgentMoreLayersNeurons(observationInfo,actionInfo,Ts)

statePath = [
    featureInputLayer(observationInfo.Dimension(1),'Normalization','zscore','Name','observation')
    fullyConnectedLayer(300,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(600,'Name','fc2')
    additionLayer(2,'Name','add')
    reluLayer('Name','relu4')
    fullyConnectedLayer(1,'Name','fc5')
    ];

actionPath = [
    featureInputLayer(actionInfo.Dimension(1),'Normalization','zscore','Name','action')
    fullyConnectedLayer(600, 'Name', 'fc6')
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
    fullyConnectedLayer(300,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(600,'Name','fc2')
    reluLayer('Name','relu2')
    fullyConnectedLayer(1,'Name','fc4')
    tanhLayer('Name','relu3')
    ];
actorNetwork = dlnetwork(actorNetwork);

actorOptions = rlOptimizerOptions('LearnRate',1e-4,'GradientThreshold',1);
actor = rlContinuousDeterministicActor(actorNetwork,observationInfo,actionInfo);


% agentOptions = rlDDPGAgentOptions(...
%     'SampleTime',Ts,...
%     'CriticOptimizerOptions',criticOptions,...
%     'ActorOptimizerOptions',actorOptions,...
%     'ExperienceBufferLength',1e4,...
%     'MiniBatchSize', 128, ...
%     'NumStepsToLookAhead',3, ...
%      'NumEpoch',2,...
%      'DiscountFactor', 0.98);

agentOptions = rlDDPGAgentOptions(...
    'SampleTime',Ts,...
    'CriticOptimizerOptions',criticOptions,...
    'ActorOptimizerOptions',actorOptions,...
    'ExperienceBufferLength',1e4,...
    'MiniBatchSize', 128,...
    'DiscountFactor', 0.98);

agentOptions.NoiseOptions.Variance = 0.6;
agentOptions.NoiseOptions.VarianceDecayRate = 1e-3;

% agentOptions.TargetUpdateMethod = 'smoothing';
% agentOptions.TargetSmoothFactor = 1e-3;  % Adjust as needed


%GaussianActionNoise OrnsteinUhlenbeckActionNoise
% agentOptions.NoiseOptions = rl.option.OrnsteinUhlenbeckActionNoise;
% agentOptions.NoiseOptions.Variance = 0.6;
% agentOptions.NoiseOptions.VarianceDecayRate = 1e-2;


agent = rlDDPGAgent(actor,critic,agentOptions);
% agent.ExperienceBuffer
% new_buffer = rlReplayMemory(observationInfo,actionInfo,15000);
% agent.ExperienceBuffer = new_buffer;
% agent.ExperienceBuffer

% agent = rlPPOAgent(actor,critic,agentOptions);

end










