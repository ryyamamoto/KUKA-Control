function N = TrajectoryGenerator(Tsei,Tsci,Tscf,Tceg,Tces,k)
% 
% ========================  Trajectory Generator  =========================
% 
% Param:  Tsei = initial configuration of end-effector,
%         Tsci = initial configuration of cube,
%         Tscf = Desired final configuration of cube,
%         Tceg = configuration of end-effector relative to cube while 
%                grasping,
%         Tces = standoff configuration of the end-effector above the cube 
%                (before and after grasping) relative to cube,
%         k    = number of trajectory reference configurations per 
%                0.01 seconds (integer with a value >1)
% Return: N    = representation of configurations in time t with each 
%                reference point being a transformation matrix Tse and
%                gripper state (0 open or 1 close),
%         => rep_config_ver#.csv file representing the N configurations above
%         -> format: r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,grip 
% 
% This function generates the trajectory motion of the youBot end-effector
% based on initial, gripping, and standoff positions of the robot-cube 
% system. The maneuvers generated has the robot pick up a cube in its 
% initial position and place it down at its final position. Function 
% assumes all configuration values are given in meters and radians.
%
% ===============================  Example  ===============================
%
% Input:
%
% clear all; close all; clc;
% Tsei = [1 0 0 0.1992; 0 1 0 0; 0 0 1 0.7535; 0 0 0 1];
% Tsci = [1 0 0 1; 0 1 0 0; 0 0 1 0.025; 0 0 0 1];
% Tscf = [0 1 0 0; -1 0 0 -1; 0 0 1 0.025; 0 0 0 1];
% Tceg = [-0.7071 0 0.7071 0.018; 0 1 0 0; -0.7071 0 -0.7071 0; 0 0 0 1];
% Tces = [-0.7071 0 0.7071 0; 0 1 0 0; -0.7071 0 -0.7071 0.1; 0 0 0 1];
% k = 1;
% N = TrajectoryGenerator(Tsei,Tsci,Tscf,Tceg,Tces,k)
% 
% Output: 
%
% N = (926x13 double) 
%   => with row values (r11,r12,r13,r21,r22,r23,r31,r32,r33,px,py,pz,grip)
%   => 'rep_config_ver#.csv' file also created with values of N, placed  
%      in folder 'MAE204_Project_Trajectories'
%

%% Initialize function variables
grip = 0;          % Gripper state (0-open / 1-closed)
poly = 5;          % Always use 5th order polynomial for smooth motion
t    = 4;          % Trajectory time in seconds
ts   = 1;          % Lift/Drop time in seconds
n    = t*k/0.01;   % Number of steps for each maneuver
ns   = ts*k/0.01;  % Number of steps for each lift/drop maneuver
N    = [];

%% 1. Move end-effector from initial to standoff (gripper open 0)
traj = CartesianTrajectory(Tsei,Tsci*Tces,t,n,poly);
for i=1:n
    [R,p] = TransToRp(traj{i});
    N = [N; [reshape(R',1,[]) p' grip]];
end

%% 2. Move gripper down to grasping position
traj = CartesianTrajectory(Tsci*Tces,Tsci*Tceg,ts,ns,poly);
for i=1:ns
    [R,p] = TransToRp(traj{i});
    N = [N; [reshape(R',1,[]) p' grip]];
end

%% 3. Close the gripper - takes 0.63s (#steps = 0.63*k/0.01)
grip = 1;
for i=1:0.63*k/0.01
    [R,p] = TransToRp(Tsci*Tceg);
    N = [N; [reshape(R',1,[]) p' grip]];
end

%% 4. Move gripper back to standoff configuration (with gripper closed 1)
traj = CartesianTrajectory(Tsci*Tceg,Tsci*Tces,ts,ns,poly);
for i=1:ns
    [R,p] = TransToRp(traj{i});
    N = [N; [reshape(R',1,[]) p' grip]];
end

%% 5. Move end-effector and cube to standoff above final position
traj = CartesianTrajectory(Tsci*Tces,Tscf*Tces,t,n,poly);
for i=1:n
    [R,p] = TransToRp(traj{i});
    N = [N; [reshape(R',1,[]) p' grip]];
end

%% 6. Move gripper down to dropping position
traj = CartesianTrajectory(Tscf*Tces,Tscf*Tceg,ts,ns,poly);
for i=1:ns
    [R,p] = TransToRp(traj{i});
    N = [N; [reshape(R',1,[]) p' grip]];
end

%% 7. Open the gripper - takes 0.63s (#steps = 0.63*k/0.01)
grip = 0;
for i=1:0.63*k/0.01
    [R,p] = TransToRp(Tscf*Tceg);
    N = [N; [reshape(R',1,[]) p' grip]];
end

%% 8. Return to standoff position above final cube position
traj = CartesianTrajectory(Tscf*Tceg,Tscf*Tces,ts,ns,poly);
for i=1:ns
    [R,p] = TransToRp(traj{i});
    N = [N; [reshape(R',1,[]) p' grip]];
end

%% Create csv with values of N
folderName = 'MAE204_Project_Results/Trial';
folderNum = 0;
while isfolder([folderName num2str(folderNum)])
    folderNum = folderNum+1;
end
if ~isfolder([folderName num2str(folderNum)])
    mkdir([folderName num2str(folderNum)]);
end
csvwrite([folderName num2str(folderNum) '/rep_config.csv'],N);

end