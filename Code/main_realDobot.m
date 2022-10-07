%% Dobot Setup
clear all;
clc;
close all;
rossshutdown;

rosinit;
dobot = DobotMagician();

%% Real Dobot Run

q{1} = [1.3292 1.0195 0.7656 0]; % Position of Red Condiment

q{2} = [1.52 0.94 0.81 0]; % Position of Yellow Condiment

q{3} = [1.69 0.9871 0.8054 0]; % Position of Brown Condiment

for i = 1:1:3

    q1 = [0 pi/4 pi/4 0];
    q2 = q{i};
    qMatrix = jtraj(q1,q2,50);

    jointTarget = [0,pi/4,pi/4,0]; % Initial Joint Pose
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states'); 
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    send(targetJointTrajPub,targetJointTrajMsg);
    pause(2);


    jointTarget = [qMatrix(50,1:3),0]; 
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    send(targetJointTrajPub,targetJointTrajMsg);

    pause(3);

end