%% Dobot Setup
clear all;
clc;
close all;
rosshutdown;

rosinit;
dobot = DobotMagician();

%% Real Dobot Run

q0 = [0 pi/4 pi/4 0];

q1 = [-0.6354 -0.0229 -0.2447 0];

qC{1} = [-1.4373 0.9005 1.1431 0]; % Position of Red Condiment

qC{2} = [1.52 0.94 0.81 0]; % Position of Yellow Condiment

qC{3} = [1.69 0.9871 0.8054 0]; % Position of Brown Condiment

qE{1} = [0.4075 1.0751 0.6979 0];

qE{2} = [0.4066 0.9648 0.6229 0];

qE{3} = [0.3811 0.8260 0.5593 0];



jointTarget = q0; % Initial Joint Pose
[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;
send(targetJointTrajPub,targetJointTrajMsg);
pause(2);

qI = q0;


for i = 1:1:3

    qMatrix = jtraj(qI,q1,50);

    jointTarget = [qMatrix(50,1:3),0]; 
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    send(targetJointTrajPub,targetJointTrajMsg);

    qMatrix1 = jtraj(q1,qC{i},50);

    jointTarget = [qMatrix1(50,1:3),0]; 
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    send(targetJointTrajPub,targetJointTrajMsg);
    onOff = 1;
    openClose = 1;
    dobot.PublishToolState(onOff,openClose);

    jointTarget = [qMatrix(50,1:3),0]; 
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    send(targetJointTrajPub,targetJointTrajMsg);

    qMatrix2 = jtraj(q1,qE{i},50);

    jointTarget = [qMatrix2(50,1:3),0]; 
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    send(targetJointTrajPub,targetJointTrajMsg);

    onOff = 1;
    openClose = 0;
    dobot.PublishToolState(onOff,openClose);

    qI = qE{i};

end