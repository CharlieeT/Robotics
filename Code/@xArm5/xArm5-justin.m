classdef xArm5 < handle
    %% DobotMagician
    % This class is based on the DobotMagician. 
    % URL: https://en.dobot.cn/products/education/magician.html
    % 
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties (Access =public)
        plyFileNameStem = 'xArm5';
        
        model;
        %> defaultRealQ 
        defaultRealQ  = [0,0,0,0,0];
    end

    methods (Access = public) 
%% Define robot Function  
        function self = xArm5
            self.CreateModel();            
            %self.PlotAndColourRobot(self);            
            %self.model.animate(self.RealQToModelQ(self.defaultRealQ))
        end

%% Create the robot model
        function CreateModel(self)       
            L1 = Link('d',0.267,    'a',0,      'alpha',-pi/2,  'offset',0, 'qlim',[deg2rad(-360),deg2rad(360)]);
            L2 = Link('d',0,        'a',0.28948866,  'alpha',0,      'offset',-1.3849179, 'qlim',[deg2rad(-118),deg2rad(120)]);
            L3 = Link('d',0,        'a',0.351158796,  'alpha',0,      'offset',2.7331843, 'qlim',[deg2rad(-225),deg2rad(11)]);
            L4 = Link('d',0,        'a',0.076,      'alpha',-pi/2,  'offset', -1.3482664, 'qlim',[deg2rad(-97),deg2rad(180)]);
            L5 = Link('d',0.097,      'a',0,      'alpha',0,      'offset',0, 'qlim',[deg2rad(-360),deg2rad(360)]);

            self.model = SerialLink([L1 L2 L3 L4 L5],'name','xArm5');
            self.model.base = self.model.base * transl(0,0.6,0.736);
        end   

%% Test Move DobotMagician
    function TestMovexArm5(self)
            qPath = jtraj(self.model.qlim(:,1)',self.model.qlim(:,2)',50);                       
            for i = 1:50                
                self.model.animate(self.RealQToModelQ(qPath(i,:)));
%                 hold on;
%                 trplot(self.model.fkine(self.RealQToModelQ(qPath(i,:))));
                pause(0.2);
            end
        end
    end

    methods(Static)
%% RealQToModelQ
        % Convert the real Q to the model Q
        function modelQ = RealQToModelQ(realQ)
            modelQ = realQ;
            modelQ(3) = xArm5.ComputeModelQ3GivenRealQ2and3( realQ(2), realQ(3) );
            modelQ(4) = pi - realQ(2) - modelQ(3);    
        end
        
%% ModelQ3GivenRealQ2and3
        % Convert the real Q2 & Q3 into the model Q3
        function modelQ3 = ComputeModelQ3GivenRealQ2and3(realQ2,realQ3)
            modelQ3 = pi/2 - realQ2 + realQ3;
        end
        
%% ModelQToRealQ
        % Convert the model Q to the real Q
        function realQ = ModelQToRealQ( modelQ )
            realQ = modelQ;
            realQ(3) = xArm5.ComputeRealQ3GivenModelQ2and3( modelQ(2), modelQ(3) );
        end
        
%% RealQ3GivenModelQ2and3
        % Convert the model Q2 & Q3 into the real Q3
        function realQ3 = ComputeRealQ3GivenModelQ2and3( modelQ2, modelQ3 )
            realQ3 = modelQ3 - pi/2 + modelQ2;
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
