classdef Collision < handle
    properties
    end
    methods(Static)%Class use static methods
        function self = Collision()
        end
        %% Check for Collision
        function isCollision = CheckCollision(robot, sphereCenter, radius)
            
            %     pause(0.1)
            tr = robot.fkine(robot.getpos);
            endEffectorToCenterDist = sqrt(sum((sphereCenter-tr(1:3,4)').^2));
            if endEffectorToCenterDist <= radius
                %         disp('A collision detected');
                isCollision = 1;
            else
                %         disp(['SAFE: End effector to sphere centre distance (', num2str(endEffectorToCenterDist), 'm) is more than the sphere radius, ' num2str(radius), 'm']);
                isCollision = 0;
            end
            
        end
        %% Create Epllisoid around robot for collision detection
        function createObstacle()
            centerPoint = [0,0,0];
            radii = [0.5,0.5,0.5];
            [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1),
            radii(2), radii(3) );
            sphere = surf(X,Y,Z);
        end
        %% Get algebraic distance
        function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)
            algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                + ((points(:,3)-centerPoint(3))/radii(3)).^2;
        end
    end
end
