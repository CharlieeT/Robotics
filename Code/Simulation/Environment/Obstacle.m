classdef Obstacle < handle
    properties
        ObstaclePose;
        updatedPoints;
        ObstacleVertexCount;
        ObstacleVerts;
        midPoint;
        Obstacle_h;

    end
   methods
       function self = Obstacle(ObstaclePose)
                self.ObstaclePose = ObstaclePose;
                self.location(self.ObstaclePose);
                
       end
       function location(self,ObstaclePose)
        % After saving in blender then load the triangle mesh
        [f,v,data] = plyread('Obstacle.ply','tri');

        % Get vertex count
        self.ObstacleVertexCount = size(v,1);

        % Move center point to origin
        self.midPoint = sum(v)/self.ObstacleVertexCount;
        self.ObstacleVerts = v - repmat(self.midPoint,self.ObstacleVertexCount,1);


        % Scale the colours to be 0-to-1 (they are originally 0-to-255
        %vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

        % Then plot the trisurf
        self.Obstacle_h = trisurf(f,self.ObstacleVerts(:,1),self.ObstacleVerts(:,2), self.ObstacleVerts(:,3) ...
            ,'EdgeColor','none','EdgeLighting','none');

        self.ObstaclePose = ObstaclePose;
        self.updatedPoints = [self.ObstaclePose * [self.ObstacleVerts,ones(self.ObstacleVertexCount,1)]']';

        % Now update the Vertices
        self.Obstacle_h.Vertices = self.updatedPoints(:,1:3);

       end
       
       function move(self,NewPose)
      
               
                self.ObstaclePose = NewPose;
                % Transform the vertices
                self.updatedPoints = [self.ObstaclePose * [self.ObstacleVerts,ones(self.ObstacleVertexCount,1)]']';
    
                % Update the mesh vertices in the patch handle
                self.Obstacle_h.Vertices = self.updatedPoints(:,1:3);
                drawnow();   
       end
   end  
end