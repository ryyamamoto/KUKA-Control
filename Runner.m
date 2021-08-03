% Ryan Yamamoto - A14478430
% Script Wrapper for MAE 204 Final Project
% 
% This script utilizes all methods need to control a 5R mobile youBot. 
% Running this script utilizes the NextState, TrajectoryGenerator, and 
% FeedbackControl functions to move youBot to interact, pick up, and put 
% down a cube in simulation through CoppeliaSim.
%
% ===============================  Default  ===============================
%
% % Set block and robot configuration parameters (x,y,theta)
% cbi    = [1  0     0];                  % cube initial configuration
% cbf    = [0 -1 -pi/2];                  % desired cube final position               
% % actual youBot initial configuration
% bot    = [0.5 -0.5  0.5 ...             % chassis (phi,x,y)
%             0    0 -0.5 -0.3  0.6 ...   % joints (J1,J2,J3,J4,J5)
%             0    0    0    0    0];     % Wheels and gripper (W1,W2,W3,W4,gripper)
% % reference youBot initial trajectory
% botRef = [0  0  0 ...                   % chassis (phi,x,y)
%           0  0 -1 -0.4 0 ...            % joints (J1,J2,J3,J4,J5)
%           0  0  0    0 0];              % Wheels and gripper (W1,W2,W3,W4,gripper)
% 
% % Controller gains
% Kp = 1.8; Ki = 0;
% 

clear; close all; clc

disp('=================  MAE 204 Final Project Script  ==================')

%% Initialize system parameters

% Set block and robot configuration parameters (x,y,theta)
cbi    = [1  0     0];                  % cube initial configuration
cbf    = [0 -1 -pi/2];                  % desired cube final position               
% actual youBot initial configuration
bot    = [0.5 -0.5  0.5 ...             % chassis (phi,x,y)
            0    0 -0.5 -0.3  0.6 ...   % joints (J1,J2,J3,J4,J5)
            0    0    0    0    0];     % Wheels and gripper (W1,W2,W3,W4,gripper)
% reference youBot initial trajectory
botRef = [0  0  0 ...                   % chassis (phi,x,y)
          0  0 -1 -0.4 0 ...            % joints (J1,J2,J3,J4,J5)
          0  0  0    0 0];              % Wheels and gripper (W1,W2,W3,W4,gripper)

% Controller gains
Kp = 1.8; Ki = 0;

% Pick-up and drop-down variables
k = 1;
a = 3/4*pi; % Grip angle when gripping cube - about y_e (radians)
standoff = 10e-2; % Standoff position above cube center (meters)
maxU = 20;


%% Get user input to change default parameters
changeDef = 1;
while changeDef
    defaultRes = upper(input('Change the default values? Y/N [N]:','s'));
    if defaultRes == 'Y'
        
        fprintf('\nBlank reponses will set as default. Default setup can be seen \n')
        fprintf('through the "help Runner" command.\n\n')
        
        % Change initial cube configuration
        changeCbi = 1;
        while changeCbi
            cbiRes = input('Change cube initial configuration? (x,y,theta vector):');
            if isempty(cbiRes)
                changeCbi = 0;
            elseif length(cbiRes) == 3
                cbi = cbiRes;
                changeCbi = 0;
            else
                disp('Invalid response.');
            end
        end
        
        % Change cube final configuration
        changeCbf = 1;
        while changeCbf
            cbfRes = input('Change cube final configuration? (x,y,theta vector):');
            if isempty(cbfRes)
                changeCbf = 0;
            elseif length(cbfRes) == 3
                cbf = cbfRes;
                changeCbf = 0;
            else
                disp('Invalid response.');
            end
        end
        
        % Change youBot actual initial configuration
        changeBot = 1;
        while changeBot
            botRes = input('Change youBot actual initial configuration? (13 var vector):');
            if isempty(botRes)
                changeBot = 0;
            elseif length(botRes) == 13
                bot = botRes;
                changeBot = 0;
            else
                disp('Invalid response.');
            end
        end
        
        % Change youBot reference initial configuration
        changeBotRef = 1;
        while changeBotRef
            botRefRes = input('Change youBot reference initial configuration? (13 var vector):');
            if isempty(botRefRes)
                changeBotRef = 0;
            elseif length(botRefRes) == 13
                botRef = botRefRes;
                changeBotRef = 0;
            else
                disp('Invalid response.');
            end
        end
        
        % Change controller gains
        changeGain = 1;
        while changeGain
            GainRes = input('Change controller gain? (Kp,Ki) [(1.8,0)]:');
            if isempty(GainRes)
                changeGain = 0;
            elseif length(GainRes) == 2
                Kp = GainRes(1);
                Ki = GainRes(2);
                changeGain = 0;
            else
                disp('Invalid response.');
            end
        end
        
        changeDef = 0;
    elseif defaultRes=='N'
        changeDef = 0;
    elseif isempty(defaultRes)
        changeDef = 0;
    else
        disp('Invalid response.');
    end
end  

%% Define system matrices
fprintf('\n========================  Running Script  =========================')
fprintf('\nInitializing configuration space matrices...')

% Define body Jacobian from joint angles
Blist = [[0  0 1       0 0.033 0]', ...
         [0 -1 0 -0.5076     0 0]', ...
         [0 -1 0 -0.3526     0 0]', ...
         [0 -1 0 -0.2176     0 0]', ...
         [0  0 1       0     0 0]' ]; 
     
% Define robot matrices to determine end-effector initial configuration
Tb0  = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.0026; 0 0 0 1];
M0e  = [1 0 0 0.0330; 0 1 0 0; 0 0 1 0.6546; 0 0 0 1];

% Define initial arm orientation -> e-e in terms of {0}
T0ei = M0e;
for i = 1:5 
    T0ei = T0ei*MatrixExp6(VecTose3(Blist(:,i)*botRef(i+3)));
end

wref = [0 0 1] * botRef(3);
pref = [botRef(1) botRef(2) 0.0963]';
Tsb  = RpToTrans(MatrixExp3(VecToso3(wref)), pref);
Tsei = Tsb*Tb0*T0ei; % Reference initial configuration Tse

% Define cube configurations
wcbi = [0 0 1] * cbi(3);
pcbi = [cbi(1) cbi(2) 2.5e-2]';
Tsci = RpToTrans(MatrixExp3(VecToso3(wcbi)),pcbi);
wcbf = [0 0 1] * cbf(3);
pcbf = [cbf(1) cbf(2) 2.5e-2]';
Tscf = RpToTrans(MatrixExp3(VecToso3(wcbf)),pcbf);
 
% Define end-effector configurations in {c}
Rg = MatrixExp3(VecToso3([0 1 0]'*a)); % Grip orientation rotation
Tceg = RpToTrans(Rg,[(0.043-0.025) 0 0]');
Tces = RpToTrans(Rg,[0 0 standoff]');

% Variables for base Jacobian
r = 0.0475;      % radius of all wheels
l = 0.235;       % distance from center of cart to wheel axles
w = 0.15;        % distance along axles from center to wheel
F = r/4*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1];
F6 = [zeros(2,4); F; zeros(1,4)];

fprintf('.........done\n')

%% Generate reference trajectory
fprintf('Generating reference trajectories...')
N = TrajectoryGenerator(Tsei,Tsci,Tscf,Tceg,Tces,k);
fprintf('.................done\n')

%% Loop through configurations N
fprintf('Looping through configurations...')
dt = 0.01;
count = 0;
X = bot; Xerr = []; Xint = 0;
for k=1:length(N)-1
    
    % Get velocity twist X,Xd,Xdn
    Xsb = RpToTrans(MatrixExp3(VecToso3([0 0 1]*X(k,1))),[X(k,2);X(k,3);0.0963]);
    T0e = M0e;
    for i = 1:5 
        T0e = T0e*MatrixExp6(VecTose3(Blist(:,i)*X(k,i+3)));
    end
    Xk  = Xsb*Tb0*T0e;
    
    Xd  = RpToTrans(reshape(N(k,1:9),3,3)',N(k,10:12)');
    Xdn = RpToTrans(reshape(N(k+1,1:9),3,3)',N(k+1,10:12)');
    [V, Xerr(k,:), Xint] = FeedbackControl(Xk,Xd,Xdn,Xint,Kp*eye(6),Ki*eye(6),dt);
    
    % Get angular wheel and joint velocities
    Je = [Adjoint(TransInv(T0e)*TransInv(Tb0))*F6 JacobianBody(Blist,X(k,4:8)')];
    uth = pinv(Je,1e-2)*V;
    U   = [uth(5:end)' uth(1:4)'];
    
    % Calculate next state configuration
    X(k+1,:) = [NextState(X(k,1:12),U,dt,maxU) N(k+1,13)];
    
    % Test joint limits and adjust
    viol = testJointLimits(X(k+1,:));
    if ~isempty(viol)
        for j = viol
            Je(:,j) = zeros(6,1);
        end
        uth = pinv(Je,1e-2)*V;
        U   = [uth(5:end)' uth(1:4)'];
        X(k+1,:) = [NextState(X(k,1:12),U,dt,maxU) N(k+1,13)];
    end
    
    if(k/(length(N)-1)*20)>count
        count = count+1;
        fprintf('.')
    end
        
end
fprintf('done\n')

%% Plot error twist Xerr
fprintf('Plotting error twist...')
fig = figure(1);
hold on; box on; grid on;
time = (1:length(Xerr)) * 0.01;
count = 1;

% Plot angular velocities
yyaxis left
for i = 1:3
    plot(time,Xerr(:,i),'Linewidth',2)
    
    if((i/6)*30)>count
        count = count +1;
        fprintf('.....')
    end
end
ylabel('Angular Velocity (rad/s)','FontSize',16)

% Plot linear velocities
yyaxis right
for i = 4:6
    plot(time,Xerr(:,i),'Linewidth',2)
    
    if((i/6)*30)>count
        count = count +1;
        fprintf('.....')
    end
end
ylabel('Linear Velocity (m/s)','FontSize',16)

legend('wx','wy','wz','vx','vy','vz')
title('Error Twist Over Time with Joint Limits', 'FontSize', 20)
xlabel('Time (s)','FontSize',16)

fprintf('done\n')

%% Write Xerr and configuration X values to csv
fprintf('Writing to csv files...')

folderName = 'MAE204_Project_Results/Trial';
folderNum = 0;
while isfolder([folderName num2str(folderNum)])
    folderNum = folderNum+1;
end
folderNum = folderNum-1;
if ~isfolder([folderName num2str(folderNum)])
    mkdir([folderName num2str(folderNum)]);
end

% Write resulting configurations X
csvwrite([folderName num2str(folderNum) '/config.csv'],X);

% Write error twist file Xerr
csvwrite([folderName num2str(folderNum) '/error_twist.csv'],Xerr);

% Save figure
saveas(fig,[folderName num2str(folderNum) '/ErrorPlot.png'])

fprintf('..............................done\n')

fprintf(['\nFiles can be found in Trial ' num2str(folderNum) '!\n\n'])

fprintf('==========================  End Script  ===========================\n')
