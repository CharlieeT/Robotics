%% Modified UR3 Robot to use for DoBot
% Li-Ting (Charlie) Tsai
% sID: 13336209
% 28/09/2022   (Good)!

classdef xArm5 < handle
    properties
        %> Robot model
        model;
        
        %> workspace
        workspace = [-1.5 1.5 -1.5 1.5 0.0 2];    % Good for Demo 
      
    end
    
    methods%% Class for xArm5 robot simulation
function self = xArm5
    self.GetxArm5();
    self.PlotAndColourRobot();
end
        %% GetxArm5Robot
        % Given a name (optional), create and return a xArm5 robot model
        function GetxArm5(self)
            pause(0.001);

            L1 = Link('d',0.267,   'a',0,           'alpha',0    );      % Base 
            L2 = Link('d',0,       'a',0,           'alpha',-pi/2);      % 
            L3 = Link('d',0,       'a',0.289489,    'alpha',0    );      % 
            L4 = Link('d',0,       'a',0.351159,    'alpha',0    );      % 
            L5 = Link('d',0.097,   'a',0.076,       'alpha',-pi/2);      % 

            L1.qlim = [-360 360]*pi/180;
            L2.qlim = [-118 120]*pi/180;
            L3.qlim = [-225  11]*pi/180;
            L4.qlim = [-97  180]*pi/180;
            L5.qlim = [-360 360]*pi/180;
            
            L2.offset = -1.3849179;
            L3.offset =  2.7331843;
            L4.offset = -1.3482664;

            self.model = SerialLink([L1 L2 L3 L4 L5],'name','xArm5');
            self.model.base = self.model.base * transl(0,0.6,0.736);
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 

        function PlotAndColourRobot(self)%robot,workspace)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['xArm5Link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
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
