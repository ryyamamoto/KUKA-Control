function varargout = FeedbackControl(X,Xd,Xdn,Xint,Kp,Ki,dt)
% 
% ===========================  Feedback Control  ==========================
% 
% Param:  X    = current actual end-effector configuration,
%         Xd   = current reference end-effector configuration,
%         Xdn  = end-effector configuration at the next step,
%         Xint = integral error over time before step dt,
%         Kp   = proportional gain matrix,
%         Ki   = integral gain matrix,
%         dt   = timestep size between reference trajectories (in seconds)
% Return: V    = commanded end-effector twist expressed in the end-effector 
%                frame {e},
%         Xerr = error twist for the configuration after time step dt,
%         Xi   = total estimated integral error being summed up over time
% 
% This function calculates the task-space feedforward plus feedback control
% law for a given maneuver. Function assumes all input parameters are given
% in terms of meters, radians, and seconds.
%
% ===============================  Example  ===============================
%
% Input:
%
% clear all; close all; clc;
% X   = [0.17 0 0.985 0.387; 0 1 0 0; -0.985 0 0.170 0.57; 0 0 0 1];
% Xd  = [0 0 1 0.5; 0 1 0 0; -1 0 0 0.5; 0 0 0 1];
% Xdn = [0 0 1 0.6; 0 1 0 0; -1 0 0 0.3; 0 0 0 1];
% [Kp,Ki] = deal(zeros(6));
% dt = 0.01;
% V = FeedbackControl(X,Xd,Xdn,Kp,Ki,dt);
%
% Output: 
% 
% V = [0 0 0 21.4 0 6.45]'
%

%% Calculate error twist Xerr
Xerr = se3ToVec(MatrixLog6(TransInv(X)*Xd));

%% Calculate an estimate of the integral of the error
Xi = Xint + Xerr*dt;

%% Calculate feedforward reference twist Vd
Vd = se3ToVec((1/dt)*MatrixLog6(TransInv(Xd)*Xdn));

%% Calculate end-effector twist Ve
V = Adjoint(TransInv(X)*Xd)*Vd+Kp*Xerr+Ki*Xi;

varargout{1} = V;
varargout{2} = Xerr';
varargout{3} = Xi;

end