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
        %% Create Prism around robot for collision detection
        function wallPoints = createWall()
            [Y,Z] = meshgrid(-1.5:0.01:2,0.1:0.01:1.6);
            sizeMat = size(Y);
            X = repmat(-2,sizeMat(1),sizeMat(2));
            wallPoints = [X(:),Y(:),Z(:)];
            
            %cubeAtOigin_h = plot3(wallPoints(:,1),wallPoints(:,2),wallPoints(:,3),'r.');
        end
        %% Create Prism around robot work area for collision detection
        function wallPoints = createWall2()
            [Y,Z] = meshgrid(-0.2:0.01:0.2,0.6:0.01:1);
            sizeMat = size(Y);
            X = repmat(0.2,sizeMat(1),sizeMat(2));
            wallPoints = [X(:),Y(:),Z(:)];
            x(1:1681,1) = -0.8;
            y(1:1681,1) = 0;
            z(1:1681,1) = -0.8;
            z2(1:1681,1) = -0.4;
            f = [x,y,z2];
            g = [x,y,z];
            wallPoints = [ wallPoints; wallPoints * rotz(pi/2); wallPoints * rotz(pi); wallPoints * rotz(3*pi/2);wallPoints*roty(pi/2)-f;wallPoints * roty(pi/2)-g;];
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
