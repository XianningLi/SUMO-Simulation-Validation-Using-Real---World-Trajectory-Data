function plotLanderVehicleTrajectory(ax, experiences, env, name)
% plotLanderVehicleTrajectory plots the time history of states of the
% lander vehicle in the Train PPO Agent for a Lander Vehicle example.
%
% ax is a MATLAB axes object.
% experiences is the output of the sim command.
% env is the environment object.
% name is the name of the observation.

% Copyright 2023 The MathWorks Inc.

arguments
    ax (1,1) matlab.graphics.axis.Axes
    experiences struct
    env (1,1) LanderVehicle
    name string {mustBeMember(name,["x", "y", "dx", "dy", "theta", "dtheta", "landing"])}
end

observations = [experiences.Observation];
ts = [observations.states];
obsInfo = getObservationInfo(env);
numObs = obsInfo.Dimension(1);

hold(ax, "on");

for ct = 1:numel(ts)
    obs = reshape(ts(ct).Data, numObs, ts(ct).Length)' .* [getBounds(env)' 1];

    switch lower(name)
        case "x"
            plot(ax, ts(ct).Time, obs(:,1));
            xlabel(ax,"Time (s)");
            ylabel(ax,"x");
            title(ax,"x Position (m)");

        case "y"
            plot(ax, ts(ct).Time, obs(:,2));
            xlabel(ax,"Time (s)");
            ylabel(ax,"y");
            title(ax,"y Position (m)");

        case "dx"
            plot(ax, ts(ct).Time, obs(:,3));
            xlabel(ax,"Time (s)");
            ylabel(ax,"dx");
            title(ax,"x Velocity (m/s)");

        case "dy"
            plot(ax, ts(ct).Time, obs(:,4));
            xlabel(ax,"Time (s)");
            ylabel(ax,"dy");
            title(ax,"y Velocity (m/s)");

        case "theta"
            plot(ax, ts(ct).Time, obs(:,5));
            xlabel(ax,"Time (s)");
            ylabel(ax,"theta");
            title(ax,"Angle (rad)");

        case "dtheta"
            plot(ax, ts(ct).Time, obs(:,6));
            xlabel(ax,"Time (s)");
            ylabel(ax,"dtheta");
            title(ax,"Angular Velocity (rad/s)");

        case "landing"
            stairs(ax, ts(ct).Time, obs(:,7));
            xlabel(ax,"Time (s)");
            ylabel(ax,"Value");
            title(ax,{"Landing Flag";"Airborne (0), Soft Landing (1)";"Rough Landing (-1)"});
    end
end
end