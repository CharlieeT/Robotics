classdef Coffee < handle
    properties
        CoffeePose;
        updatedPoints;
        CoffeeVertexCount;
        CoffeeVerts;
        midPoint;
        Coffee_h;

    end
   methods
       function self = Coffee(CoffeePose)
                self.CoffeePose = CoffeePose;
                self.location(self.CoffeePose);
                
       end
       function location(self,CoffeePose)
        % After saving in blender then load the triangle mesh
        [f,v,data] = plyread('Coffee.ply','tri');

        % Get vertex count
        self.CoffeeVertexCount = size(v,1);

        % Move center point to origin
        self.midPoint = sum(v)/self.CoffeeVertexCount;
        self.CoffeeVerts = v - repmat(self.midPoint,self.CoffeeVertexCount,1);


        % Scale the colours to be 0-to-1 (they are originally 0-to-255
        vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

        % Then plot the trisurf
        self.Coffee_h = trisurf(f,self.CoffeeVerts(:,1),self.CoffeeVerts(:,2), self.CoffeeVerts(:,3) ...
            ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');

        self.CoffeePose = CoffeePose;
        self.updatedPoints = [self.CoffeePose * [self.CoffeeVerts,ones(self.CoffeeVertexCount,1)]']';  

        % Now update the Vertices
        self.Coffee_h.Vertices = self.updatedPoints(:,1:3);

       end
       
       function move(self,NewPose)
      
               
                self.CoffeePose = NewPose;
                % Transform the vertices
                self.updatedPoints = [self.CoffeePose * [self.CoffeeVerts,ones(self.CoffeeVertexCount,1)]']';
    
                % Update the mesh vertices in the patch handle
                self.Coffee_h.Vertices = self.updatedPoints(:,1:3);
                drawnow();   
       end
   end  
end
