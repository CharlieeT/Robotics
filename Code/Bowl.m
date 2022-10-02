classdef Bowl < handle
    properties
        BowlPose;
        updatedPoints;
        BowlVertexCount;
        BowlVerts;
        midPoint;
        Bowl_h;

    end
   methods
       function self = Bowl(BowlPose)
                self.BowlPose = BowlPose;
                self.location(self.BowlPose);
                
       end
       function location(self,BowlPose)
        % After saving in blender then load the triangle mesh
        [f,v,data] = plyread('bowl.ply','tri');

        % Get vertex count
        self.BowlVertexCount = size(v,1);

        % Move center point to origin
        self.midPoint = sum(v)/self.BowlVertexCount;
        self.BowlVerts = v - repmat(self.midPoint,self.BowlVertexCount,1);


        % Scale the colours to be 0-to-1 (they are originally 0-to-255
        %vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

        % Then plot the trisurf
        self.Bowl_h = trisurf(f,self.BowlVerts(:,1),self.BowlVerts(:,2), self.BowlVerts(:,3), 'EdgeColor','none','EdgeLighting','none');

        self.BowlPose = BowlPose;
        self.updatedPoints = [self.BowlPose * [self.BowlVerts,ones(self.BowlVertexCount,1)]']';  

        % Now update the Vertices
        self.Bowl_h.Vertices = self.updatedPoints(:,1:3);

       end
       
       function move(self,NewPose)
      
               
                self.BowlPose = NewPose;
                % Transform the vertices
                self.updatedPoints = [self.BowlPose * [self.BowlVerts,ones(self.BowlVertexCount,1)]']';
    
                % Update the mesh vertices in the patch handle
                self.Bowl_h.Vertices = self.updatedPoints(:,1:3);
                drawnow();   
       end
   end  
end
