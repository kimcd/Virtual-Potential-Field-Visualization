%% SCRIPT_virtualPotFieldViz
% This is a visualization of the attractive, repulsive and
% combined potential fields. It is also the visualization of the 
% trajectory of a robot within the field and the workspace. 
%
% HOW TO USE: 
% 1. Alter the starting, goal and obstacle positions under "Initialize 
% Goal, Obstacles & Field Parameters" subsection. 
%
% 2. Alter "zetaAtt" and "zetaRep" to alter how strongly the robot is
% either attracted to the goal or repulsed by the obstacles. 
%
% 3. "threshDist" is the max. distance between the robot and obstacle
% before the obstacle begins to exert a repulsive force on the robot.
%
% i.e.: 
%       if distance (b/w robotPosition & obstaclePosition) <= threshDist,
%           then obstacle repels robot
%       else
%           obstacle does not exert force on robot
%       end
%
% 4. "alpha" is essentially the maximum robot velocity
%
% 5. Use the "Rotate 3D" icon in figures 2 and 3 to see the 3d surface!
%
% 6. For now, you will have have to force stop (ctrl + c) when robot is
% stuck!!!
%       
% SPECIAL CASES:
% Example of robot getting stuck in local minima: 
% qGoal = [2;2]; & qObs = {[2;1.5],[1.2;0.8],[1;1.2],[0.5;1]};
% 
% Example of robot overshooting goal and getting stuck: 
% qGoal = [2;2]; & qObs = {[2;1.5],[1.2;0.8],[1;1.5],[0.5;1]};
%
% C. Kim, 24.DEC.2018, JHUAPL

%TODO
% How to stop simulation when stuck in local minima?
% How to get color gradient on combined field surface plot?
% 

%% 
clear all
close all
fig1Flag = false; % attractive potential field plot
fig2Flag = false; % repulsive potential field plot

%% Initialize Goal, Obstacles & Field Parameters
qCurr = [0;0]; %starting point
qGoal = [2;2]; %goal location
qObs = {[0.5;1],[1;1.5],[1.2;0.8],[2;1.5],[1.5;2]}; %local minima case
zetaAtt = 1; %attractive force constant
zetaRep = .1; %repuslive force constant
threshDist = .5; %obstacle threshold distance
alpha = 1; % maximum robot velocity

[x,y] = meshgrid(0:0.05:4 , 0:0.05:4); %mesh to compute surface plots

%% Attractive FIELD (NOT force!)
%computes the attractive field using the mesh
zAtt=ones(size(x,1),size(x,2));

for row = 1:size(x,1)
    for col = 1:size(x,2)
        q = [x(row,col);y(row,col)];
        zAtt(row,col) = 0.5 * zetaAtt * norm(q - qGoal)^2;
    end
end

%% Repulsive FIELD (NOT force!)
%computes the repulsive field using the mesh
zRep=zeros(size(x,1),size(x,2));

for row = 1:size(x,1)
    for col = 1:size(x,2)
        
        q = [x(row,col);y(row,col)];
        
        for i = 1:numel(qObs)
            
            rhoQ = norm(q - qObs{i});
            
            if rhoQ <= threshDist
                zRep_i = 0.5 * zetaRep * (1/rhoQ - 1/threshDist)^2;
            else
                zRep_i = 0;
            end
            
            zRep (row,col) = zRep(row,col)+zRep_i;
            
        end
        
    end
end

%% Field Visualizations
% attractive field
if fig1Flag
fig1 = figure(1);
axs1 = axes('Parent',fig1);
surf(axs1,x,y,zAtt) %attractive potential field. dip = goal
title(axs1,'Attractive Field')
end

%repulsive field
if fig2Flag
fig2 = figure(2);
axs2 = axes('Parent',fig2);
hold(axs2,'on')
surf(axs2,x,y,zRep)
title(axs2,'Repulsive Field')
zlim(axs2,[0 4])
end

%combined field
fig3 = figure(3);
axs3 = axes('Parent',fig3);
hold(axs3,'on')
zTot = zAtt + zRep; %total z
surf(axs3,x,y,zTot)
title(axs3,'Combined Field')

%initialize fig4 
fig4 = figure(4);
axs4 = axes('Parent',fig4);
hold(axs4,'on')
plot(axs4,qCurr(1),qCurr(2),'kx');

%% Path Planning and Plotting
dt = 0.01; %simulation timestep. leave this

continueFlag = true;

while continueFlag
    %attractive force to goal
    fAtt = zetaAtt * (qGoal - qCurr); 
    
    %repulsive force to obstacles
    fRep =[0;0]; %initialize and reset repulsive force
    for i = 1:numel(qObs)
        
        rhoQ = norm(qCurr - qObs{i});
        
        if rhoQ <= threshDist
            fRep_i = zetaRep * (1/rhoQ - 1/threshDist)*(1/rhoQ^2)*(qCurr - qObs{i})/norm(qCurr-qObs{i});
        else
            fRep_i = 0;
        end
        fRep = fRep+fRep_i;
    end
    
    fTot = fAtt + fRep; %total force exerted on robot
    qCurr = qCurr + alpha * (fTot/norm(fTot) )*dt; %robot movement
       
    %find field values to plot robot on combined field surface plot (fig3)
    zAtt = 0.5 * zetaAtt * norm(qCurr - qGoal)^2;
    zRep = 0;
    
    for i = 1:numel(qObs)
        
        rhoQ = norm(qCurr - qObs{i});
        
        if rhoQ <= threshDist
            zRep_i = 0.5 * zetaRep * (1/rhoQ - 1/threshDist)^2;
        else
            zRep_i = 0;
        end
        zRep = zRep + zRep_i;
    end
    
    zTot = zAtt + zRep;
    
    %plot 
    plot(axs4,qCurr(1),qCurr(2),'k.') %plot trajectory
    plot(axs4,qGoal(1),qGoal(2),'gx') %plot goal
    
    plot3(axs3,qCurr(1),qCurr(2),zTot,'g.') %plot trajectory on surface plot
    zlim(axs3,[0 4]) %limit z-axis on axs3 (otherwise peaks go to infinity)
    
    for i = 1:numel(qObs)
        plot(axs4,qObs{i}(1),qObs{i}(2),'rx') %plot obstacles in w-space
    end
    
    axis(axs4,[0 4 0 4])
    drawnow
    
    if norm(qCurr-qGoal)<.05
        continueFlag = false; %stop when within range of goal
    end
    
    pause(dt)
    
end


