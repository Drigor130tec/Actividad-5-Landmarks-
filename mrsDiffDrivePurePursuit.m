%% EXAMPLE: Differential drive vehicle following waypoints using the 
% Pure Pursuit algorithm
%
% Copyright 2018-2019 The MathWorks, Inc.

%% Define Vehicle
R = 0.1;                % Wheel radius [m]
L = 0.5;                % Wheelbase [m]
dd = DifferentialDrive(R,L);

%% Simulation parameters
sampleTime = 0.1;               % Sample time [s]
tVec = 0:sampleTime:90;         % Time array

%initPose = [0;0;0];             % Initial pose (x y theta)
initPose = [5;1;0];
%initPose = [4;1:0];
%initPose = [10:9:180];

pose = zeros(3,numel(tVec));    % Pose matrix
pose(:,1) = initPose;

% Define waypoints
%waypoints = [0,0; 2,2; 4,2; 2,4; 0.5,3];
%waypoints = [1,9; 3,9; 3,10; 4,9; 5,10; 4,10; 3,10; 6,11; 5,12; 5,10.6; 6,11; 7,11; 7,8; 8,9; 7,12; 11,6; 12,6; 12,0; 10,0; 10,1; 10,0; 7,0; 6,2; 6,4; 11,6; 10,7; 6,5; 6,4; 6,5; 5,6; 2,6; 3,7; 4,7; 3,7; 2,6; 1,8; 2,9; 1,9; 1,8];
%waypoints = [4,1; 6,3; 8,3; 6,1; 2,1; 0,3; 2,3; 4,1; 4,5; 6,3; 6,5; 8,5; 6,7; 8,9; 6,9; 6,11; 4,9; 2,11; 2,9; 0,9; 2,7; 0,5; 2,5; 2,3; 4,5; 5,6; 5,8; 3,8; 3,6; 5,6]
waypoints = [5,1; 3,3; 3,5; 5,7; 7,7 ; 8,6; 7,5; 6,6; 7,5; 8,5; 7,5; 8,6; 10,9; 8,11; 6,11; 4,9; 10,9; 8,6; 9,5; 9,3; 7,1; 5,1];

% Create visualizer
viz = Visualizer2D;
viz.hasWaypoints = true;

%% Pure Pursuit Controller
controller = controllerPurePursuit;
controller.Waypoints = waypoints;
controller.LookaheadDistance = 0.45;
controller.DesiredLinearVelocity = 0.55;
controller.MaxAngularVelocity = 2.2;

%% Simulation loop
close all
r = rateControl(1/sampleTime);
for idx = 2:numel(tVec) 
    % Run the Pure Pursuit controller and convert output to wheel speeds
    [vRef,wRef] = controller(pose(:,idx-1));
    [wL,wR] = inverseKinematics(dd,vRef,wRef);
    
    % Compute the velocities
    [v,w] = forwardKinematics(dd,wL,wR);
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose(:,idx-1));  % Convert from body to world
    
    % Perform forward discrete integration step
    pose(:,idx) = pose(:,idx-1) + vel*sampleTime; 
    
    % Update visualization
    viz(pose(:,idx),waypoints)
    waitfor(r);
end