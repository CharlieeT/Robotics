function placeEnvironment()
    %% Clear all ==============================================================
    clc
    clear all;
    close all;
    set(0, 'DefaultFigureWindowStyle', 'docked');    
    %% Enviroment =============================================================
    
    disp('Creating Environment, Please Wait -------- ');
    hold on
    
    % axis equal;
    
    % Load Texture Image for the Ground  ------------------------------------------------------------------
    surf([-2.6,-2.6; 2.6, 2.6],[-2.6, 2.6;-2.6, 2.6],[0.0,0.01;0.0,0.01],'CData',imread('Floor.jpg'),'FaceColor','texturemap');
    
    % Load Table ------------------------------------------------------------------------------------------
    disp('Loading the Scene.... ');
    PlaceObject("table-small.ply",[0,0,0]);
    PlaceObject("Cafe-Enviro-new.ply",[0,0,0]);
    PlaceObject("Human.ply", [0,0,0]);
    % PlaceObject("Screen-and-Estop.ply",[0,0,0]);
    
    disp('Loading Safety Equipment.... ');
    PlaceObject("Estop.ply",[0,0,0]);
    PlaceObject("FireEx.ply",[-0.5,1.5,0]);
    PlaceObject("Laser-Curtain.ply",[0,0,0]);
    PlaceObject("Fences.ply",[0,0,0]);
    PlaceObject("WarningSign.ply",[0,0,0]);
    PlaceObject("Screen-and-Estop.ply", [0,0,0]);
    PlaceObject("First-aid-kit.ply", [0,0,0]);
    PlaceObject("Camera.ply", [0,0,0]);
end