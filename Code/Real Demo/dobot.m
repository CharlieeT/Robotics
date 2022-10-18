classdef dobot < handle
    properties
    end
    methods(Static)%Class use static methods
        function self = dobot()
        end
        function dobot = start()
            rosshutdown;
            rosinit;

            dobot = DobotMagician();
            dobot.InitaliseRobot();

        end
        function move(i)
                dobot = DobotMagician();
                q0 = [0 pi/4 pi/4 0];
                q1 = [-1.1943 0.0868 -0.2077 0];
                q2 = [0.4324 0.4943 -0.2041 0];
                
                qC{1} = [-1.4373 0.9005 1.1431 0]; % Start Position of Red Condiment

                qC{2} = [-1.1111 1.0054 1.0071 0]; % Start Position of Yellow Condiment

                qC{3} = [-0.8639 1.2014 0.7282 0]; % Start Position of Brown Condiment

                qE{1} = [0.4075 1.0751 0.6979 0]; % End Position of Red Condiment

                qE{2} = [0.4066 0.9648 0.6229 0]; % End Position of Red Condiment

                qE{3} = [0.3811 0.8260 0.5593 0]; % End Position of Red Condiment


                qMatrix = jtraj(q0,q1,50);
                qMatrix2 = jtraj(q1,q2,50);
                qMatrix1 = jtraj(q1,qC{i},50);
                qMatrix3 = jtraj(q2,qE{2},50);
                qMatrix4 = jtraj(qE{2}, q2, 50);

                Movements_realDobot.move(qMatrix);
                Movements_realDobot.move(qMatrix1);

                Movements_realDobot.toolOnOff(dobot, 1);

                Movements_realDobot.move(qMatrix);
                Movements_realDobot.move(qMatrix2);
                Movements_realDobot.move(qMatrix3);

                Movements_realDobot.toolOnOff(dobot,0);

                Movements_realDobot.move(qMatrix4);
           
        end
    end
end
