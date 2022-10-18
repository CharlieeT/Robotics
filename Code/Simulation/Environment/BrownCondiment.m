classdef BrownCondiment < handle
    properties
        BrownCondimentPose;
        updatedPoints;
        BrownCondimentVertexCount;
        BrownCondimentVerts;
        midPoint;
        BrownCondiment_h;

    end
   methods
       function self = BrownCondiment(BrownCondimentPose)
                self.BrownCondimentPose = BrownCondimentPose;
                self.location(self.BrownCondimentPose);
                
       end
       function location(self,BrownCondimentPose)
        % After saving in blender then load the triangle mesh
        [f,v,data] = plyread('BrownCondiment.ply','tri');

        % Get vertex count
        self.BrownCondimentVertexCount = size(v,1);

        % Move center point to origin
        self.midPoint = sum(v)/self.BrownCondimentVertexCount;
        self.BrownCondimentVerts = v - repmat(self.midPoint,self.BrownCondimentVertexCount,1);


        % Scale the colours to be 0-to-1 (they are originally 0-to-255
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

        % Then plot the trisurf
        self.BrownCondiment_h = trisurf(f,self.BrownCondimentVerts(:,1),self.BrownCondimentVerts(:,2), self.BrownCondimentVerts(:,3) ...
            ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');

        self.BrownCondimentPose = BrownCondimentPose;
        self.updatedPoints = [self.BrownCondimentPose * [self.BrownCondimentVerts,ones(self.BrownCondimentVertexCount,1)]']';  

        % Now update the Vertices
        self.BrownCondiment_h.Vertices = self.updatedPoints(:,1:3);

       end
       
       function move(self,NewPose)
      
               
                self.BrownCondimentPose = NewPose;
                % Transform the vertices
                self.updatedPoints = [self.BrownCondimentPose * [self.BrownCondimentVerts,ones(self.BrownCondimentVertexCount,1)]']';
    
                % Update the mesh vertices in the patch handle
                self.BrownCondiment_h.Vertices = self.updatedPoints(:,1:3);
                drawnow();   
       end
   end  
end
