classdef MotomanHC10DTP < handle
    properties
        %> Robot model
        model;
        
        %>
        workspace = [-2 2 -2 2 0 2];   
               
    end
    
    methods%% Class for Motoman robot simulation
        function self = MotomanHC10DTP()

                
        % robot = 
        self.GetRobotModel();
        % robot = 
        self.PlotAndColourRobot();%robot,workspace);
        end

        %% GetRobotModel
        % Given a name (optional), create and return a Motoman robot model
        function GetRobotModel(self)
            name = "MOTOMAN HC10DTP";
        
%             L1 = Link('d',0.275,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
%             L2 = Link('d',0,'a',0.7,'alpha',-pi,'offset',-pi/2,'qlim',[deg2rad(-90),deg2rad(90)]);
%             L3 = Link('d',0,'a',0.5,'alpha',pi,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
%             L4 = Link('d',0.162,'a',0,'alpha',-pi/2,'offset',-pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);
%             L5 = Link('d',0.170,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
%             L6 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);

            L1 = Link('d',0.275,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            L2 = Link('d',0,'a',0.7,'alpha',0,'offset',pi/2,'qlim',[deg2rad(-90),deg2rad(90)]);
            L3 = Link('d',0,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
            L4 = Link('d',0.5,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            L5 = Link('d',0.162,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            L6 = Link('d',0.170,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
        
            self.model = SerialLink([L1 L2 L3 L4 L5 L6], 'name', name);
            self.model.delay = 0;
        end
        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
				[ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['MotomanHC10DTPLink',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end
        
            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace, 'floorlevel', 0);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;
        
            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end        
    end
end