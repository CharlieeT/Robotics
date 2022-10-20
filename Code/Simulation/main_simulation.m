%% Clear all ==============================================================
clc
clear all;
close all;
set(0, 'DefaultFigureWindowStyle', 'docked');

%% Enviroment =============================================================

disp('Creating Environment, Please Wait -------- ');
hold on
% 
% axis equal;

% Load Texture Image for the Ground  ------------------------------------------------------------------
surf([-2.6,-2.6; 2.6, 2.6],[-2.6, 2.6;-2.6, 2.6],[0.0,0.01;0.0,0.01],'CData',imread('Floor.jpg'),'FaceColor','texturemap');

% Load Table ------------------------------------------------------------------------------------------
disp('Loading Table.... ');
PlaceObject("table-small.ply",[0,0,0]);
PlaceObject("Cafe-Enviro-new.ply",[0,0,0]);
% PlaceObject("Screen-and-Estop.ply",[0,0,0]);
PlaceObject("Estop.ply",[0,0,0]);


% % Load Boxes ----------------------------------------------------------------------------------------
disp('Loading Boxes.... ');
PlaceObject("Redbox.ply",[-0.05,-0.01,0]);
PlaceObject("Yellowbox.ply",[0.10,-0.01,0]);
PlaceObject("Brownbox.ply",[0.01,-0.01,0]);

% Load Blocks --------------------------------------------------------------------------------------

% Original ----------------------------------------------------
disp('Loading Condiments.... ');
Red = RedCondiment(transl(0.005,-0.22,0.75));
Yellow = YellowCondiment(transl(0.085,-0.22,0.75));
Brown = BrownCondiment(transl(-0.075,-0.22,0.75));
Bowl = Bowl(transl(0.25,0,0.78));
% -------------------------------------------------------------


hold off

% Load Dobot ----------------------------------------------------------------------------------------
disp('Loading Robot.... ');
dobot = DobotM();
qInitial = dobot.q;

% Load xArm5 ----------------------------------------------------------------------------------------
xarm = xArm5;


view([160, 28]);   % Changing the camera angle 
hold off
input('Done Loading Environment! Press Enter to Start')



%% Contorl of Robot ======================================================= 


% Moving 1
T1 = dobot.model.fkine(qInitial);
T2 = transl(T1(1:3,4))*transl(0,0,0.2);

Movements.moveikcon(dobot,T2,50);
Movements.moveikcon(dobot,Red.RedCondimentPose,50);

move1 = transl(0.05,-0.22,0.95);

Movements.moveobji(dobot,move1,Red,50);
% Resolved Motion Rate Control 1
finalPosRed = [0.25, 0, 0.8];

% Movements.rmrc(dobot,finalPosRed, Red, 50);

% % Moving 2
% 
% Movements.moveikcon(dobot,Yellow.YellowCondimentPose,50);
% 
% move2 = transl(0.13,-0.22,0.95);
% 
% Movements.moveobji(dobot,move2,Yellow,50);
% 
% % Resolved Motion Rate Control 2
% 
% finalPosYellow = [0.25, 0, 0.83];
% 
% Movements.rmrc(dobot,finalPosYellow, Yellow, 50);

%% Collision Detection

% Create Wall of Detection around the table
hold on;
% Placing Obstacle
Obstacle = Obstacle(transl(-2,0,1.3));
centerPoint = Obstacle.ObstaclePose(1:3,4);
axis equal;
Movements.rmrcObj(dobot, finalPosRed,Obstacle, centerPoint, [-0.8, 0, 1.5],Red, 50);

%% Joystick Control
% setup joystick
id = 6; % Note: may need to be changed if multiple joysticks present
joy = vrjoystick(id);
caps(joy) % display joystick information

% start control
q = dobot.q;                 % Set initial robot configuration 'q'

HF = figure(1);         % Initialise figure to display robot

set(HF,'Position',[0.1 0.1 0.8 0.8]);

duration = 300;  % Set duration of the simulation (seconds)
dt = 0.15;      % Set time step for simulation (seconds)

n = 0;  % Initialise step count to zero 
tic;    % recording simulation start time
while( toc < duration)
    
    n=n+1; % increment step count

    % read joystick
    [axes, buttons, povs] = read(joy);
       
    % -------------------------------------------------------------
    % YOUR CODE GOES HERE
    % 1 - turn joystick input into an end-effector velocity command
    Kv = 0.3; % linear velocity gain
    Kw = 0.8; % angular velocity gain
    
    vx = Kv*axes(1);
    vy = Kv*axes(2);
    vz = Kv*(buttons(5)-buttons(7));
    
    wx = Kw*axes(4);
    wy = Kw*axes(3);
    wz = Kw*(buttons(6)-buttons(8));
    
    dx = [vx;vy;vz;wx;wy;wz]; % combined velocity vector
    
    % 2 - use DLS J inverse to calculate joint velocity
    lambda = 0.5;
    J = dobot.model.jacob0(q);
    Jinv_dls = inv((J'*J)+lambda^2*eye(5))*J';
    dq = Jinv_dls*dx;
    
    % 3 - apply joint velocity to step robot joint angles 
    q = q + dq'*dt;
      
    % -------------------------------------------------------------
    
    % Update plot
    dobot.model.animate(q);  
    drawnow();
    % wait until loop time elapsed
    if (toc > dt*n)
        warning('Loop %i took too much time - consider increating dt',n);
    end
    while (toc < dt*n); % wait until loop time (dt) has elapsed 
    end
end


%% Return to Original Position =========================================================================
% q1 = dobot.model.getpos;
% q2 = qInitial * transl(0,0,0.5);         % This is wrong
% 
% 
% qMatrix4 = jtraj(q1,q2,50);
% 
% for i = 1:50                                                                % Moving the robot to original pose
%     dobot.model.animate(qMatrix4(i,:));
%     drawnow();
% end





% %% algebraicDist
% function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)
% 
% algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
%               + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
%               + ((points(:,3)-centerPoint(3))/radii(3)).^2;
% end
% 

