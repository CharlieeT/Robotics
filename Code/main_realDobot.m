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

qC{1} = [-1.4373 0.9005 1.1431 0]; % Position of Red Condiment

qC{2} = [-1.1111 1.0054 1.0071 0]; % Position of Yellow Condiment

qC{3} = [-0.8639 1.2014 0.7282 0]; % Position of Brown Condiment

qE{1} = [0.4075 1.0751 0.6979 0];

qE{2} = [0.4066 0.9648 0.6229 0];

qE{3} = [0.3811 0.8260 0.5593 0];

q2 = [0.4324 0.4943 -0.2041 0];



jointTarget = q0; % Initial Joint Pose
[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;
send(targetJointTrajPub,targetJointTrajMsg);
pause(2)

qI = q0;


for i = 1:1:3

    qMatrix = jtraj(qI,q1,50);
    qMatrix2 = jtraj(q1,q2,50);

    jointTarget = [qMatrix(50,1:3),0]; 
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    send(targetJointTrajPub,targetJointTrajMsg);
    pause(2)
    qMatrix1 = jtraj(q1,qC{i},50);

    jointTarget = [qMatrix1(50,1:3),0]; 
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    send(targetJointTrajPub,targetJointTrajMsg);
    pause(2)
    onOff = 1;
    openClose = 1;
    dobot.PublishToolState(onOff,openClose);

    jointTarget = [qMatrix(50,1:3),0]; 
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    send(targetJointTrajPub,targetJointTrajMsg);
    pause(2)

    jointTarget = [qMatrix2(50,1:3),0]; 
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    send(targetJointTrajPub,targetJointTrajMsg);
    pause(2)

    qMatrix3 = jtraj(q2,qE{i},50);

    jointTarget = [qMatrix3(50,1:3),0]; 
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    send(targetJointTrajPub,targetJointTrajMsg);
    pause(2)

    qMatrix4 = jtraj(qE{i}, q2, 50);

    onOff = 0;
    openClose = 0;
    dobot.PublishToolState(onOff,openClose);

    jointTarget = [qMatrix4(50,1:3),0]; 
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    trajectoryPoint.Positions = jointTarget;
    targetJointTrajMsg.Points = trajectoryPoint;
    send(targetJointTrajPub,targetJointTrajMsg);
    pause(2)

   
    qI = q2;

end