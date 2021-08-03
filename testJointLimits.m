function viol = testJointLimits(X)
% 
% ========  Test Joint Limits - Avoid Collision and Singularities  ========
% 
% Param:  X     = joint and wheel positions after time step dt
% Return: viol  = Vector of joints limits that are violated
% 
% This function checks that each joint configuration does not result in a 
% collision or singularity. Function assumes all input parameters are given
% in terms of meters, radians, and seconds.
%
% ===============================  Example  ===============================
%
% Input:
% 
% X = [79 95 65 3 84 93 67 75 74 39 65 17 70];
% viol = testJointLimits(X)
% 
% Output:
% 
% viol = [5 6 7];
%

viol = [];

%% Define joint positions at step dt from current
j1 = X(4); j2 = X(5); j3 = X(6); j4 = X(7);

%% Define Joint limits
jlim   = 0.1;     % Singularity avoidance joint limits
j2lim  = pi*5/12; % j2 self-collision limit
j3lim  = pi*3/4;  % j3 self-collision limit
jlimcp = pi/4;    % Chassis angle max when j2 is positive
jlimcn = pi*3/4;  % Chassis angle min when j2 is negative
jc     = pi*9/16; % Chassis collision angle
jf     = pi*3/4;  % Floor collision angle

%% Avoid any possible singularities and self collision
if j2 > j2lim
    viol = [viol 6];
end

if abs(j3) < jlim || abs(j3) > j3lim
    viol = [viol 7];
end

if abs(j4) < jlim
    viol = [viol 8];
end

%% Avoid chassis and floor collision
j = [j2 j3 j4];
% Above chassis
if ((sign(j2)>0)&&(abs(j1)<jlimcp))||((sign(j2)<0)&&(abs(j1)>jlimcn))
    if (abs(sum(j)) > jc)
        indMax = find(max(j)==j,1);
        viol = [viol 5 5+indMax];
    end
else % Above floor
    if (abs(sum(j)) > jf)
        indMax = find(max(j)==j,1);
        viol = [viol 5 5+indMax];
    end
end

viol = sort(viol);
viol = unique(viol);

end