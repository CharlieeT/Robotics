%% Dobot Setup
clear all;
clc;
close all;

rosshutdown;
rosinit;

dobot = DobotMagician();

%% Real Dobot Run

q0 = [0 pi/4 pi/4 0];

q1 = [-1.1943 0.0868 -0.2077 0];

q2 = [0.4324 0.4943 -0.2041 0];

qC{1} = [-1.4373 0.9005 1.1431 0]; % Start Position of Red Condiment

qC{2} = [-1.1111 1.0054 1.0071 0]; % Start Position of Yellow Condiment

qC{3} = [-0.8639 1.2014 0.7282 0]; % Start Position of Brown Condiment

qE{1} = [0.4075 1.0751 0.6979 0]; % End Position of Red Condiment

qE{2} = [0.4066 0.9648 0.6229 0]; % End Position of Red Condiment

qE{3} = [0.3811 0.8260 0.5593 0]; % End Position of Red Condiment

qI = q0;


for i = 1:1:3

    qMatrix = jtraj(qI,q1,50);
    qMatrix2 = jtraj(q1,q2,50);
    qMatrix1 = jtraj(q1,qC{i},50);
    qMatrix3 = jtraj(q2,qE{i},50);
    qMatrix4 = jtraj(qE{i}, q2, 50);

    Movements_realDobot.move(qMatrix);
    Movements_realDobot.move(qMatrix1);

    Movements_realDobot.toolOnOff(dobot, 1);

    Movements_realDobot.move(qMatrix);
    Movements_realDobot.move(qMatrix2);
    Movements_realDobot.move(qMatrix3);

    Movements_realDobot.toolOnOff(dobot,0);

    Movements_realDobot.move(qMatrix4);
   
    qI = q2;

end
%% Joystick Control

% setup joystick
id = 1; % Note: may need to be changed if multiple joysticks present
joy = vrjoystick(id);
caps(joy) % display joystick information

%% start control
q = dobot.GetCurrentJointState;                % Set initial robot configuration 'q'

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
    J = jacob0(q);
    Jinv_dls = inv((J'*J)+lambda^2*eye(5))*J';
    dq = Jinv_dls*dx;
    
    % 3 - apply joint velocity to step robot joint angles 
    q = q + dq'*dt;
      
    % -------------------------------------------------------------
    
    % Update plot
    jointTarget = [q,0]; % Initial Joint Pose
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    send(targetJointTrajPub,targetJointTrajMsg);
    pause(2);
    % wait until loop time elapsed
    if (toc > dt*n)
        warning('Loop %i took too much time - consider increating dt',n);
    end
    while (toc < dt*n); % wait until loop time (dt) has elapsed
    end
end