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
        function createCollision(robot,q)
            centerPoint = [0,0,0];
            radii = [1,0.5,0.5];
            [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1),
            radii(2), radii(3) );
            for i = 1:4
                robot.points{i} = [X(:),Y(:),Z(:)];
                warning off
                robot.faces{i} = delaunay(robot.points{i});
                warning on;
            end
            
            tr = robot.fkine(q);
            cubePointsAndOnes = [inv(tr) * [cubePoints,ones(size(cubePoints,1),1)]']';
            updatedCubePoints = cubePointsAndOnes(:,1:3);
            algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
            pointsInside = find(algebraicDist < 1);
            display(['2.9: There are now ', num2str(size(pointsInside,1)),' points inside']);
            
            for i = 1: size(tr,3)
                cubePointsAndOnes = [inv(tr(:,:,i)) *
                    [cubePoints,ones(size(cubePoints,1),1)]']';
                updatedCubePoints = cubePointsAndOnes(:,1:3);
                algebraicDist = GetAlgebraicDist(updatedCubePoints, centerPoint, radii);
                pointsInside = find(algebraicDist < 1);
                display(['2.10: There are ', num2str(size(pointsInside,1)),' points inside the',num2str(i),'th ellipsoid']);
            end
        end
        %% Get algebraic distance
        function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)
            algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                + ((points(:,3)-centerPoint(3))/radii(3)).^2;
        end
    end
end
