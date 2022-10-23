function joystick1(Obstacle, joy)

    [axes, buttons, povs] = read(joy);
    if double(buttons(1)) == 1
        Obstacle.ObstaclePose(3,4) = Obstacle.ObstaclePose(3,4)-0.01;
        down = Obstacle.ObstaclePose;
        Obstacle.move(down);
        drawnow();
    elseif double(buttons(2)) == 1
        Obstacle.ObstaclePose(2,4) = Obstacle.ObstaclePose(2,4)-0.01;
        right = Obstacle.ObstaclePose;
        Obstacle.move(right);
        drawnow();
    elseif double(buttons(3)) == 1
        Obstacle.ObstaclePose(2,4) = Obstacle.ObstaclePose(2,4)+0.01;
        left = Obstacle.ObstaclePose;
        Obstacle.move(left);
        drawnow();
    elseif double(buttons(4)) == 1
        Obstacle.ObstaclePose(3,4) = Obstacle.ObstaclePose(3,4)+0.01;
        up = Obstacle.ObstaclePose;
        Obstacle.move(up);
        drawnow();
    elseif double(buttons(5)) == 1
        Obstacle.ObstaclePose(1,4) = Obstacle.ObstaclePose(1,4)-0.01;
        forward = Obstacle.ObstaclePose;
        Obstacle.move(forward);
        drawnow();
    elseif double(buttons(6)) == 1
        Obstacle.ObstaclePose(1,4) = Obstacle.ObstaclePose(1,4)+0.01;
        backward = Obstacle.ObstaclePose;
        Obstacle.move(backward);
        drawnow();
    end
    
    pause(0.05);
end