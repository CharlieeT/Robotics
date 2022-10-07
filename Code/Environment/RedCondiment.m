classdef RedCondiment < handle
    properties
        RedCondimentPose;
        updatedPoints;
        RedCondimentVertexCount;
        RedCondimentVerts;
        midPoint;
        RedCondiment_h;

    end
   methods
       function self = RedCondiment(RedCondimentPose)
                self.RedCondimentPose = RedCondimentPose;
                self.location(self.RedCondimentPose);
                
       end
       function location(self,RedCondimentPose)
        % After saving in blender then load the triangle mesh
        [f,v,data] = plyread('RedCondiment.ply','tri');

        % Get vertex count
        self.RedCondimentVertexCount = size(v,1);

        % Move center point to origin
        self.midPoint = sum(v)/self.RedCondimentVertexCount;
        self.RedCondimentVerts = v - repmat(self.midPoint,self.RedCondimentVertexCount,1);


        % Scale the colours to be 0-to-1 (they are originally 0-to-255
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

        % Then plot the trisurf
        self.RedCondiment_h = trisurf(f,self.RedCondimentVerts(:,1),self.RedCondimentVerts(:,2), self.RedCondimentVerts(:,3) ...
            ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');

        self.RedCondimentPose = RedCondimentPose;
        self.updatedPoints = [self.RedCondimentPose * [self.RedCondimentVerts,ones(self.RedCondimentVertexCount,1)]']';  

        % Now update the Vertices
        self.RedCondiment_h.Vertices = self.updatedPoints(:,1:3);

       end
       
       function move(self,NewPose)
      
               
                self.RedCondimentPose = NewPose;
                % Transform the vertices
                self.updatedPoints = [self.RedCondimentPose * [self.RedCondimentVerts,ones(self.RedCondimentVertexCount,1)]']';
    
                % Update the mesh vertices in the patch handle
                self.RedCondiment_h.Vertices = self.updatedPoints(:,1:3);
                drawnow();   
       end
   end  
end
