%% Clear all ==============================================================
clc
clear all;
close all;
set(0, 'DefaultFigureWindowStyle', 'docked');

%% Enviroment =============================================================

disp('Creating Environment, Please Wait -------- ');
hold on

% axis equal;

% Load Texture Image for the Ground  ------------------------------------------------------------------
surf([-2.6,-2.6; 2.6, 2.6],[-2.6, 2.6;-2.6, 2.6],[0.0,0.01;0.0,0.01],'CData',imread('Floor.jpg'),'FaceColor','texturemap');

% Load Table ------------------------------------------------------------------------------------------
disp('Loading the Scene.... ');
PlaceObject("table-small.ply",[0,0,0]);
PlaceObject("Cafe-Enviro-new.ply",[0,0,0]);
PlaceObject("Human.ply", [0,0,0]);
% PlaceObject("Screen-and-Estop.ply",[0,0,0]);

disp('Loading Safety Equipment.... ');
PlaceObject("Estop.ply",[0,0,0]);
PlaceObject("FireEx.ply",[-0.5,1.5,0]);
PlaceObject("Laser-Curtain.ply",[0,0,0]);
PlaceObject("Fences.ply",[0,0,0]);

% % Load Boxes ----------------------------------------------------------------------------------------
disp('Loading Boxes.... ');
PlaceObject("Redbox.ply",[-0.05,-0.01,0]);
PlaceObject("Yellowbox.ply",[0.10,-0.01,0]);
PlaceObject("Brownbox.ply",[0.01,-0.01,0]);


%% Loading Sauce & Bowl ----------------------------------------------------
disp('Loading Condiments & Bowl & Coffee.... ');
Red = RedCondiment(transl(0.005,-0.22,0.75));
% Red{2} = RedCondiment(transl(0.005,-0.22,0.77));
% Red{1} = RedCondiment(transl(0.005,-0.22,0.79));
Yellow = YellowCondiment(transl(0.085,-0.22,0.75));
% Yellow{2} = YellowCondiment(transl(0.085,-0.22,0.77));
% Yellow{1} = YellowCondiment(transl(0.085,-0.22,0.79));
Brown = BrownCondiment(transl(-0.075,-0.22,0.75));
% Brown{2} = BrownCondiment(transl(-0.075,-0.22,0.77));
% Brown{1} = BrownCondiment(transl(-0.075,-0.22,0.79));
Bowl = Bowl(transl(0.25,0,0.78));
Obstacle = Obstacle(transl(0,-0.25,1.3));
Coffee = Coffee(transl(1.1, -0.4, 0.81));

hold off

%% Loading Robots ----------------------------------------------------------------------------------------
disp('Loading Robots.... ');
dobot = DobotM();
qInitial = dobot.q;

xarm = xArm5;

view([-130, 27]);   % Changing the camera angle 
input('Done Loading Environment! Press Enter to Start')


%% Contorl of Robot ======================================================= 
hold on;
% Moving 1
T1 = dobot.model.fkine(qInitial);
T2 = transl(T1(1:3,4))*transl(0,0,0.2);

steps = 20;         % Dont do below 20
dobotlinkNum = 5;

Movements.moveikcon(dobot,dobotlinkNum,T2,steps,Obstacle);

% Moving 1

Movements.moveikcon(dobot,dobotlinkNum,Red.RedCondimentPose,steps,Obstacle);

move1 = transl(0.05,-0.22,0.95);

Movements.moveobji(dobot,dobotlinkNum,move1,Red,steps,Obstacle);

% Resolved Motion Rate Control 1
finalPosRed = [0.25, 0, 0.8];

Movements.rmrc(dobot,dobotlinkNum,finalPosRed, Red, steps, Obstacle);

% Moving 2
 
Movements.moveikcon(dobot,dobotlinkNum,Yellow.YellowCondimentPose,steps,Obstacle);

move2 = transl(0.13,-0.22,0.95);
 
Movements.moveobji(dobot,dobotlinkNum,move2,Yellow,steps,Obstacle);
 
%Resolved Motion Rate Control 2
 
finalPosYellow = [0.25, 0, 0.83];
 
Movements.rmrc(dobot,dobotlinkNum,finalPosYellow, Yellow, steps,Obstacle);

% Moving 3
brownI = [-1.9199    0.9436    0.9928   -0.3657    1.4835];
Movements.moveikcon(dobot,dobotlinkNum,dobot.model.fkine(brownI),steps,Obstacle);

move3 = transl(-0.075,-0.22,0.95);
 
Movements.moveobji(dobot,dobotlinkNum,move3,Brown,steps,Obstacle);
 
%Resolved Motion Rate Control 3
 
finalPosBrown = [0.25, 0, 0.86];
 
Movements.rmrc(dobot,dobotlinkNum,finalPosBrown, Brown, steps,Obstacle);

% Go to finish position;

finishQ = deg2rad([-90 5 90 0 0]);

Movements.moveikcon(dobot,dobotlinkNum, dobot.model.fkine(finishQ), steps,Obstacle);

%% xArm5 Movement
xarmSteps = 80;  % Dont do below 20
xArmlinkNum = 5;               % debug: 6  if using the xarm5 with 6 link

% Moving Bowl with 3 sauces in it
bowlpose = deg2rad([120, 40.7, -90.9, 48.3, 270]);%, -0.8]);           % Bowl Start Pose   [-240, 55, -98, 40, 60, 0]

Movements.moveikcon(xarm,xArmlinkNum,xarm.model.fkine(bowlpose),xarmSteps, Obstacle);

finalPosBowl = [0.6, -1.25, 1.25]; % Top of Counter bar                % [1.2, -0.75, 0.9]

Movements.rmrc2(xarm,xArmlinkNum,finalPosBowl,Bowl,Red,Yellow,Brown,xarmSteps);


% Moving Coffee
coffeepose = deg2rad([21.8,    16.6,    33.8,   22.2,    360]);
Movements.moveikcon(xarm,xArmlinkNum,dobot.model.fkine(coffeepose),xarmSteps,Obstacle);

% movecoffee = transl(-0.075,-0.22,0.95);
%  
% Movements.moveobji(xarm,xArmlinkNum,movecoffee,Coffee,xarmSteps,Obstacle);
 
% Resolved Motion Rate Control 3
 
finalPosCoffee = [0.5, -1.25, 1.2];
 
Movements.rmrc3(xarm,xArmlinkNum,finalPosCoffee,Coffee,xarmSteps);

input("Operation Completed! Press Enter to Exit Program...");

%% Collision Detection

% % Create Wall of Detection around the table
% hold on;
% % Placing Obstacle
% Obstacle = Obstacle(transl(-2,0,1.3));
% centerPoint = Obstacle.ObstaclePose(1:3,4);
% axis equal;
% Movements.rmrcObj(dobot, finalPosRed,Obstacle, centerPoint, [-0.8, 0, 1.5],Red, steps);

%% Joystick Control
% % setup joystick
% id = 6;               % Note: may need to be changed if multiple joysticks present
% joy = vrjoystick(id);
% caps(joy) % display joystick information
% 
% % start control
% q = dobot.q;                 % Set initial robot configuration 'q'
% 
% HF = figure(1);         % Initialise figure to display robot
% 
% set(HF,'Position',[0.1 0.1 0.8 0.8]);
% 
% duration = 300;  % Set duration of the simulation (seconds)
% dt = 0.15;      % Set time step for simulation (seconds)
% 
% n = 0;  % Initialise step count to zero 
% tic;    % recording simulation start time
% while( toc < duration)
%     
%     n=n+1; % increment step count
% 
%     % read joystick
%     [axes, buttons, povs] = read(joy);
%        
%     Kv = 0.3; % linear velocity gain
%     Kw = 0.8; % angular velocity gain
%     
%     vx = Kv*axes(1);
%     vy = Kv*axes(2);
%     vz = Kv*(buttons(5)-buttons(7));
%     
%     wx = Kw*axes(4);
%     wy = Kw*axes(3);
%     wz = Kw*(buttons(6)-buttons(8));
%     
%     dx = [vx;vy;vz;wx;wy;wz]; % combined velocity vector
%     
%     % 2 - use DLS J inverse to calculate joint velocity
%     lambda = 0.5;
%     J = dobot.model.jacob0(q);
%     Jinv_dls = inv((J'*J)+lambda^2*eye(5))*J';
%     dq = Jinv_dls*dx;
%     
%     % 3 - apply joint velocity to step robot joint angles 
%     q = q + dq'*dt;
%       
%     % -------------------------------------------------------------
%     
%     % Update plot
%     dobot.model.animate(q);  
%     drawnow();
%     % wait until loop time elapsed
%     if (toc > dt*n)
%         warning('Loop %i took too much time - consider increating dt',n);
%     end
%     while (toc < dt*n); % wait until loop time (dt) has elapsed 
%     end
% end 
%%
