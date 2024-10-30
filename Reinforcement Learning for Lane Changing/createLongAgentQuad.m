function agent = createLongAgentQuad(observationInfo,actionInfo,Ts)
% Helper function for creating a DDPG agent.

% Copyright 2020 The MathWorks, Inc.


% Observation and action paths
obsPath = featureInputLayer(observationInfo.Dimension(1),Name="obsInLyr");
actPath = featureInputLayer(actionInfo.Dimension(1),Name="actInLyr");

% Common path
commonPath = [
    concatenationLayer(1,2,Name="concat")
    quadraticLayer
    fullyConnectedLayer(1,Name="value", ...
        BiasLearnRateFactor=0,Bias=0)
    ];

criticNet = dlnetwork;
criticNet = addLayers(criticNet,obsPath);
criticNet = addLayers(criticNet,actPath);
criticNet = addLayers(criticNet,commonPath);


criticNet = connectLayers(criticNet,"obsInLyr","concat/in1");
criticNet = connectLayers(criticNet,"actInLyr","concat/in2");

criticNet = initialize(criticNet);

critic = rlQValueFunction(criticNet, ...
    observationInfo, actionInfo, ...
    ObservationInputNames="obsInLyr", ...
    ActionInputNames="actInLyr");

getValue(critic,{rand(observationInfo.Dimension)},{rand(actionInfo.Dimension)})


actorNet = [
    featureInputLayer(observationInfo.Dimension(1))
    fullyConnectedLayer(actionInfo.Dimension(1), ...
        BiasLearnRateFactor=0,Bias=0)
    ];


actorNet = dlnetwork(actorNet);
actorNet = initialize(actorNet);

actor = rlContinuousDeterministicActor(actorNet,observationInfo,actionInfo);

getAction(actor,{rand(observationInfo.Dimension)})

agent = rlDDPGAgent(actor,critic);

agent.AgentOptions.SampleTime = Ts;
agent.AgentOptions.ExperienceBufferLength = 1e5;
agent.AgentOptions.MiniBatchSize = 32;
agent.AgentOptions.NoiseOptions.StandardDeviation = 0.3;
agent.AgentOptions.NoiseOptions.StandardDeviationDecayRate = 1e-5;
agent.AgentOptions.ActorOptimizerOptions.LearnRate = 1e-3;
agent.AgentOptions.ActorOptimizerOptions.GradientThreshold = 1;
agent.AgentOptions.CriticOptimizerOptions.LearnRate = 5e-3;
agent.AgentOptions.CriticOptimizerOptions.GradientThreshold = 1;








