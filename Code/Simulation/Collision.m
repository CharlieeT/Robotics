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
        function wallPoints = createWall()
            [Y,Z] = meshgrid(-1:0.01:1,0.1:0.01:1.5);
            sizeMat = size(Y);
            X = repmat(1,sizeMat(1),sizeMat(2));
            wallPoints = [X(:),Y(:),Z(:)];
            
            wallPoints = [ wallPoints; wallPoints * rotz(pi/2)...
                ; wallPoints * rotz(pi); wallPoints * rotz(3*pi/2)];
            %cubeAtOigin_h = plot3(wallPoints(:,1),wallPoints(:,2),wallPoints(:,3),'r.');
        end
        %% Check for collision
        function collision = isCollision(wallPoints, objectCenter, objectRadii)
            algebraicDist = GetAlgebraicDist(wallPoints, objectCenter, objectRadii);
            pointsInside = find(algebraicDist<=1);
            if (pointsInside > 0)
                collision = 1;
            else
                collision = 0;
            end
            
            function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)
            algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                + ((points(:,3)-centerPoint(3))/radii(3)).^2;
            end
        end
        %% Get algebraic distance
        
    end
end
