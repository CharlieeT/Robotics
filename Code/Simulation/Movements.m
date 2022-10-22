classdef Movements < handle
    properties
    end
    methods(Static)%Class use static methods
        function self = Movements()
        end
        %% Movement funtions by use inverse kinematics
        function moveikcon(robot,T,steps,Obstacle)
            points = Collision.createWall2();
            radii = [0.1 0.1 0.1];
            id = 5; % Note: may need to be changed if multiple joysticks present
            joy = vrjoystick(id);
            q1 = robot.model.getpos;
            q2 = robot.model.ikcon(T,q1);
            s = lspb(0,1,steps);
            qMatrix = nan(steps,5);
            %Calculate trajectory using Trapezoidal methods
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
                animate(robot.model,qMatrix(i,:));
                joystick1(Obstacle, joy);
                centerPoint = Obstacle.ObstaclePose(1:3,4);
                if Collision.isCollision(points, centerPoint, radii) == 1
                    disp("Object Detected");
                end
                drawnow();
            end
            function joystick1(Obstacle, joy)

            [axes, buttons, povs] = read(joy);
            if double(buttons(1)) == 1
                Obstacle.ObstaclePose(3,4) = Obstacle.ObstaclePose(3,4)-0.01;
                down = Obstacle.ObstaclePose;
                Obstacle.move(down);
                drawnow();
            elseif double(buttons(2)) == 1
                Obstacle.ObstaclePose(2,4) = Obstacle.ObstaclePose(2,4)-0.01;
                right = Obstacle.ObstaclePose;
                Obstacle.move(right);
                drawnow();
            elseif double(buttons(3)) == 1
                Obstacle.ObstaclePose(2,4) = Obstacle.ObstaclePose(2,4)+0.01;
                left = Obstacle.ObstaclePose;
                Obstacle.move(left);
                drawnow();
            elseif double(buttons(4)) == 1
                Obstacle.ObstaclePose(3,4) = Obstacle.ObstaclePose(3,4)+0.01;
                up = Obstacle.ObstaclePose;
                Obstacle.move(up);
                drawnow();
            elseif double(buttons(5)) == 1
                Obstacle.ObstaclePose(1,4) = Obstacle.ObstaclePose(1,4)-0.01;
                forward = Obstacle.ObstaclePose;
                Obstacle.move(forward);
                drawnow();
            elseif double(buttons(6)) == 1
                Obstacle.ObstaclePose(1,4) = Obstacle.ObstaclePose(1,4)+0.01;
                backward = Obstacle.ObstaclePose;
                Obstacle.move(backward);
                drawnow();
            end

            pause(0.05);
        end
        end
        %% Movement objects and robot using inverse kinematics
        function moveobji(robot,T,obj,steps, Obstacle)
            points = Collision.createWall2();
            radii = [0.1 0.1 0.1];
            % Get vertex count
            id = 5; % Note: may need to be changed if multiple joysticks present
            joy = vrjoystick(id);
            q1 = robot.model.getpos();
            q2 = robot.model.ikcon(T,q1);
            s = lspb(0,1,steps);
            qMatrix = nan(steps,5);
            %Calculate trajectory using Trapezoidal methods
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
                animate(robot.model,qMatrix(i,:));
                newPose = robot.model.fkine(qMatrix(i,:));
                obj.move(newPose); 
                joystick1(Obstacle, joy);
                centerPoint = Obstacle.ObstaclePose(1:3,4);
                if Collision.isCollision(points, centerPoint, radii) == 1
                    disp("Object Detected");
                end
                drawnow();
            end
            function joystick1(Obstacle, joy)

            [axes, buttons, povs] = read(joy);
            if double(buttons(1)) == 1
                Obstacle.ObstaclePose(3,4) = Obstacle.ObstaclePose(3,4)-0.01;
                down = Obstacle.ObstaclePose;
                Obstacle.move(down);
                drawnow();
            elseif double(buttons(2)) == 1
                Obstacle.ObstaclePose(2,4) = Obstacle.ObstaclePose(2,4)-0.01;
                right = Obstacle.ObstaclePose;
                Obstacle.move(right);
                drawnow();
            elseif double(buttons(3)) == 1
                Obstacle.ObstaclePose(2,4) = Obstacle.ObstaclePose(2,4)+0.01;
                left = Obstacle.ObstaclePose;
                Obstacle.move(left);
                drawnow();
            elseif double(buttons(4)) == 1
                Obstacle.ObstaclePose(3,4) = Obstacle.ObstaclePose(3,4)+0.01;
                up = Obstacle.ObstaclePose;
                Obstacle.move(up);
                drawnow();
            elseif double(buttons(5)) == 1
                Obstacle.ObstaclePose(1,4) = Obstacle.ObstaclePose(1,4)-0.01;
                forward = Obstacle.ObstaclePose;
                Obstacle.move(forward);
                drawnow();
            elseif double(buttons(6)) == 1
                Obstacle.ObstaclePose(1,4) = Obstacle.ObstaclePose(1,4)+0.01;
                backward = Obstacle.ObstaclePose;
                Obstacle.move(backward);
                drawnow();
            end

            pause(0.05);
        end
        end
        %% Resolve Motion Rate Control
        function rmrc(robot, finalPos, obj, steps, Obstacle)
            points = Collision.createWall2();
            radii = [0.1 0.1 0.1];
            id = 5;
            joy = vrjoystick(id);
            deltaT = 0.05;                                        % Discrete time step
            x = zeros(3,50);
            s = lspb(0,1,50);                                 % Create interpolation scalar
            initialPos = robot.model.fkine(robot.model.getpos());
            for i = 1:50
                x(1,i) = initialPos(1,4)*(1-s(i)) + s(i)*finalPos(1);
                x(2,i) = initialPos(2,4)*(1-s(i)) + s(i)*finalPos(2);
                x(3,i) = initialPos(3,4)*(1-s(i)) + s(i)*finalPos(3);
                x(4,i) = 0;
                x(5,i) = 0;
            end

            qMatrix1 = nan(steps,5);

            q0 = robot.model.getpos;
            T = robot.model.fkine(q0);
            qMatrix1(1,:) = robot.model.ikcon(T,q0);                 % Solve for joint angles

            for i = 1:steps-1
                xdot = (x(:,i+1) - x(:,i))/deltaT;                             % Calculate velocity at discrete time step
                J = robot.model.jacob0(qMatrix1(i,:));            % Get the Jacobian at the current state
                J = J(1:5,1:5);
                qdot = inv(J)*xdot;                             % Solve velocitities via RMRC
                qMatrix1(i+1,:) =  qMatrix1(i,:) + deltaT*qdot';      % Update next joint state
                robot.model.animate(qMatrix1(i,:));
                newPose1 = robot.model.fkine(qMatrix1(i,:));
                obj.move(newPose1);
                joystick1(Obstacle,joy);
                centerPoint = Obstacle.ObstaclePose(1:3,4);
                if Collision.isCollision(points, centerPoint, radii) == 1
                    disp("Object Detected");
                end
                drawnow();
            end

            for i = 1:49
                robot.model.animate(qMatrix1(50-i,:));
                joystick1(Obstacle, joy);
                points = Collision.createWall2();
                centerPoint = Obstacle.ObstaclePose(1:3,4);
                if Collision.isCollision(points, centerPoint, radii) == 1
                    disp("Object Detected");
                end
                drawnow();
            end
            function joystick1(Obstacle, joy)
            [axes, buttons, povs] = read(joy);
            if double(buttons(1)) == 1
                Obstacle.ObstaclePose(3,4) = Obstacle.ObstaclePose(3,4)-0.01;
                down = Obstacle.ObstaclePose;
                Obstacle.move(down);
                drawnow();
            elseif double(buttons(2)) == 1
                Obstacle.ObstaclePose(2,4) = Obstacle.ObstaclePose(2,4)-0.01;
                right = Obstacle.ObstaclePose;
                Obstacle.move(right);
                drawnow();
            elseif double(buttons(3)) == 1
                Obstacle.ObstaclePose(2,4) = Obstacle.ObstaclePose(2,4)+0.01;
                left = Obstacle.ObstaclePose;
                Obstacle.move(left);
                drawnow();
            elseif double(buttons(4)) == 1
                Obstacle.ObstaclePose(3,4) = Obstacle.ObstaclePose(3,4)+0.01;
                up = Obstacle.ObstaclePose;
                Obstacle.move(up);
                drawnow();
            elseif double(buttons(5)) == 1
                Obstacle.ObstaclePose(1,4) = Obstacle.ObstaclePose(1,4)-0.01;
                forward = Obstacle.ObstaclePose;
                Obstacle.move(forward);
                drawnow();
            elseif double(buttons(6)) == 1
                Obstacle.ObstaclePose(1,4) = Obstacle.ObstaclePose(1,4)+0.01;
                backward = Obstacle.ObstaclePose;
                Obstacle.move(backward);
                drawnow();
            end

            pause(0.05);
        end
        end
        %% Movement using RMRC with obstacle
        function rmrcObj(robot, finalPos, obj,objStart, objEnd, cond, steps)
            points = Collision.createWall();
            radii = [0.1 0.1 0.1];
            deltaT = 0.05;                                        % Discrete time step
            x = zeros(3,50);
            s = lspb(0,1,50);                                 % Create interpolation scalar
            initialPos = robot.model.fkine(robot.model.getpos());
            for i = 1:50
                x(1,i) = initialPos(1,4)*(1-s(i)) + s(i)*finalPos(1);
                x(2,i) = initialPos(2,4)*(1-s(i)) + s(i)*finalPos(2);
                x(3,i) = initialPos(3,4)*(1-s(i)) + s(i)*finalPos(3);
                x(4,i) = 0;
                x(5,i) = 0;
            end

            qMatrix1 = nan(steps,5);

            q0 = robot.model.getpos;
            T = robot.model.fkine(q0);
            qMatrix1(1,:) = robot.model.ikcon(T,q0);                 % Solve for joint angles
            z = 0;
            for i = 1:steps-1
                xdot = (x(:,i-z+1) - x(:,i-z))/deltaT;                             % Calculate velocity at discrete time step
                J = robot.model.jacob0(qMatrix1(i-z,:));            % Get the Jacobian at the current state
                J = J(1:5,1:5);
                qdot = inv(J)*xdot;                             % Solve velocitities via RMRC
                qMatrix1(i-z+1,:) =  qMatrix1(i-z,:) + deltaT*qdot';      % Update next joint state
                
                newPose1 = [1 0 0 objStart(1)+(i*(objEnd(1)-objStart(1))/50); 0 1 0 objStart(2)+(i*(objEnd(2)-objStart(2))/50); 0 0 1 objStart(3)+(i*(objEnd(3)-objStart(3))/50); 0 0 0 1];
                obj.move(newPose1);
                newPose2 = robot.model.fkine(qMatrix1(i-z,:));
                cond.move(newPose2);
                centerPoint = obj.ObstaclePose(1:3,4);
                if Collision.isCollision(points, centerPoint, radii) == 1
                    disp("Object Detected");
                    z = z+1;
                else 
                    robot.model.animate(qMatrix1(i-z,:));
                end
                joystick1(obstacle);
                drawnow();
            end
% 
%             for i = 1:49
%                 robot.model.animate(qMatrix1(50-i,:));
%                 drawnow();
%             end
        end
        %% Movement funtions by use inverse kinematics
        function moveikine(bot,T,steps)
            
            q1 = bot.mdl.getpos();
            q2 = bot.mdl.ikcon(T,q1);
            s = lspb(0,1,steps);
            qM = nan(steps,5);
            %Calculate trajectory using Trapezoidal methods
            for i = 1:steps
                qM(i,:) = (1-s(i))*q1 + s(i)*q2;
                
                animate(bot.mdl,qM(i,:));
                drawnow();
            end
        end
        %% Resolve Motion Rate Control
        function rmrc2(robot, finalPos, obj,cond1, cond2, cond3, steps)
            deltaT = 0.05;                                        % Discrete time step
            x = zeros(3,steps);
            s = lspb(0,1,steps);                                 % Create interpolation scalar
            initialPos = robot.model.fkine(robot.model.getpos());
            for i = 1:steps
                x(1,i) = initialPos(1,4)*(1-s(i)) + s(i)*finalPos(1);
                x(2,i) = initialPos(2,4)*(1-s(i)) + s(i)*finalPos(2);
                x(3,i) = initialPos(3,4)*(1-s(i)) + s(i)*finalPos(3);
                x(4,i) = 0;
                x(5,i) = 0;
            end

            qMatrix1 = nan(steps,5);

            q0 = robot.model.getpos;
            T = robot.model.fkine(q0);
            qMatrix1(1,:) = robot.model.ikcon(T,q0);                 % Solve for joint angles

            for i = 1:steps-1
                xdot = (x(:,i+1) - x(:,i))/deltaT;                             % Calculate velocity at discrete time step
                J = robot.model.jacob0(qMatrix1(i,:));            % Get the Jacobian at the current state
                J = J(1:5,1:5);
                qdot = inv(J)*xdot;                             % Solve velocitities via RMRC
                qMatrix1(i+1,:) =  qMatrix1(i,:) + deltaT*qdot';      % Update next joint state
                robot.model.animate(qMatrix1(i,:));
                newPose1 = robot.model.fkine(qMatrix1(i,:));
                obj.move(newPose1*trotx(pi)*transl(-0.03,0.03,-0.09));
                cond1.move(newPose1*trotx(pi)*transl(-0.03,0.03,-0.08));
                cond2.move(newPose1*trotx(pi)*transl(-0.03,0.03,-0.06));
                cond3.move(newPose1*trotx(pi)*transl(-0.03,0.03,-0.04));
                drawnow();
            end

%             for i = 1:49
%                 robot.model.animate(qMatrix1(50-i,:));
%                 drawnow();
%             end
        end
                %% Resolve Motion Rate Control
        function rmrc1(robot, finalPos, steps)
            deltaT = 0.05;                                        % Discrete time step
            x = zeros(3,steps);
            s = lspb(0,1,steps);                                 % Create interpolation scalar
            initialPos = robot.model.fkine(robot.model.getpos());
            for i = 1:steps
                x(1,i) = initialPos(1,4)*(1-s(i)) + s(i)*finalPos(1);
                x(2,i) = initialPos(2,4)*(1-s(i)) + s(i)*finalPos(2);
                x(3,i) = initialPos(3,4)*(1-s(i)) + s(i)*finalPos(3);
                x(4,i) = 0;
                x(5,i) = 0;
            end

            qMatrix1 = nan(steps,5);

            q0 = robot.model.getpos;
            T = robot.model.fkine(q0);
            qMatrix1(1,:) = robot.model.ikcon(T,q0);                 % Solve for joint angles

            for i = 1:steps-1
                xdot = (x(:,i+1) - x(:,i))/deltaT;                             % Calculate velocity at discrete time step
                J = robot.model.jacob0(qMatrix1(i,:));            % Get the Jacobian at the current state
                J = J(1:5,1:5);
                qdot = inv(J)*xdot;                             % Solve velocitities via RMRC
                qMatrix1(i+1,:) =  qMatrix1(i,:) + deltaT*qdot';      % Update next joint state
                robot.model.animate(qMatrix1(i,:));
                drawnow();
            end

%             for i = 1:49
%                 robot.model.animate(qMatrix1(50-i,:));
%                 drawnow();
%             end
        end
    end
end