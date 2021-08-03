function Xf = NextState(Xi,U,dt,maxU)
% 
% ===================  Next State Kinematics Simulator  ===================
% 
% Param:  Xi   = current state of robots
%                -> format: 12 vars (3 chassis, 5 arms, 4 wheel angles),
%         U    = joint and wheel velocities (in radians)
%                -> format: 9 vars (5 arm, 4 wheels),
%         dt   = timestep size (in seconds),
%         maxU = maximum joint and wheel velocity magnitudes (in rad/s)
% Return: Xf   = next configuration state of robot - one time step later 
%                -> format: 12 vars (3 chassis, 5 arms, 4 wheel angles)
% 
% This function uses the kinematics of youBot to determine the next
% configuration - one time step later. Function assumes all input 
% parameters are given in terms of meters, radians, and seconds.
%
% ===============================  Example  ===============================
%
% Input:
%
% clear all; close all; clc;
% Xi = [0 1 0 0 0 0 0 0 0 0 0 0];
% dt = 0.01;
% U = [1 1 0.5 1 0.25 10 -10 10 10];
% maxU = 12;
% Xf = NextState(Xi,U,dt,maxU)
%
% Output: 
% 
% Xf =
%   Columns 1 through 7
%    -0.0062    1.0024   -0.0024    0.0100    0.0100    0.0050    0.0100
%   Columns 8 through 12
%     0.0025    0.1000   -0.1000    0.1000    0.1000 
%

%% Check joint and wheel velocities compared to limits
maxU = abs(maxU);
for i = 1:length(U)
    if (abs(U(i)) > maxU)
        U(i) = sign(U(i))*maxU;
    end
end

%% Seperate input parmeters by componets
q_chas = reshape(Xi(1:3),[],1);       % Chassis variables (phi,x,y)
th_arm = reshape(Xi(4:8),[],1);       % Arm position angles (radians)
th_whl = reshape(Xi(9:end),[],1);     % Wheel position angles (radians)
th_arm_dot = reshape(U(1:5),[],1);    % Arm joint velocities (rad/s)
th_whl_dot = reshape(U(6:end),[],1);  % Wheel angular velocities (rad/s)

%% Determine next step configuration via a first-order Euler step
th_arm_new = th_arm + th_arm_dot*dt;
th_whl_new = th_whl + th_whl_dot*dt;

%% Use odometry to determine new chassis configuration

% Initialize odometry variables
r = 0.0475;      % radius of all wheels
l = 0.235;       % distance from center of cart to wheel axles
w = 0.15;        % distance along axles from center to wheel

% get twist Vb = (wz,vx,vy) based on wheel velocities - Eq.13.33 in MR
F = r/4*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1];
Vb = F*th_whl_dot*dt;
wz = Vb(1); vx = Vb(2); vy = Vb(3); 

% Determine dq = (dphi_b,dx_b,dy_b) - Eq.13.35 in MR
if (Vb(1) == 0)
    dqb = Vb;
else
    dqb = [wz;(vx*sin(wz)+vy*(cos(wz)-1))/wz;(vy*sin(wz)+vx*(1-cos(wz)))/wz];
end

% Convert dqb into {s} frame
phi = q_chas(1);
dq = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)]*dqb;

% Increment position 
q_chas_new = q_chas + dq;

%% Concatenate new configuration variables
Xf = [q_chas_new' th_arm_new' th_whl_new'];

end