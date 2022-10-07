classdef YellowCondiment < handle
    properties
        YellowCondimentPose;
        updatedPoints;
        YellowCondimentVertexCount;
        YellowCondimentVerts;
        midPoint;
        YellowCondiment_h;

    end
   methods
       function self = YellowCondiment(YellowCondimentPose)
                self.YellowCondimentPose = YellowCondimentPose;
                self.location(self.YellowCondimentPose);
                
       end
       function location(self,YellowCondimentPose)
        % After saving in blender then load the triangle mesh
        [f,v,data] = plyread('YellowCondiment.ply','tri');

        % Get vertex count
        self.YellowCondimentVertexCount = size(v,1);

        % Move center point to origin
        self.midPoint = sum(v)/self.YellowCondimentVertexCount;
        self.YellowCondimentVerts = v - repmat(self.midPoint,self.YellowCondimentVertexCount,1);


        % Scale the colours to be 0-to-1 (they are originally 0-to-255
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

        % Then plot the trisurf
        self.YellowCondiment_h = trisurf(f,self.YellowCondimentVerts(:,1),self.YellowCondimentVerts(:,2), self.YellowCondimentVerts(:,3) ...
            ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');

        self.YellowCondimentPose = YellowCondimentPose;
        self.updatedPoints = [self.YellowCondimentPose * [self.YellowCondimentVerts,ones(self.YellowCondimentVertexCount,1)]']';  

        % Now update the Vertices
        self.YellowCondiment_h.Vertices = self.updatedPoints(:,1:3);

       end
       
       function move(self,NewPose)
      
               
                self.YellowCondimentPose = NewPose;
                % Transform the vertices
                self.updatedPoints = [self.YellowCondimentPose * [self.YellowCondimentVerts,ones(self.YellowCondimentVertexCount,1)]']';
    
                % Update the mesh vertices in the patch handle
                self.YellowCondiment_h.Vertices = self.updatedPoints(:,1:3);
                drawnow();   
       end
   end  
end
