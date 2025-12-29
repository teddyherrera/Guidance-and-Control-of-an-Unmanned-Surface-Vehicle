% ME4811        Introduction to Engineering System Dynamics and Control
%                Naval Postgraduate School, Monterey CA
%% Title:        MinefieldAndWaypointPlotter_v2_1
% Students:     Teddy Herrera, Joesph Young, Braden Zukowski               
% Desciption:   Plots the waypoints, mines and vehicle trajectory.
%                   Exisiting logic removes mines from planned trajectory.
% Note: Figure numbers changed to accomodate multiple headings/waypoints


%%
% clear all
% clc

% startPointA = [0,0];
% goalPointB = [200, 0];

PARAMETERS = data;
mineRing = 2*PARAMETERS.ATTACKERWEAPON.F; %Mine threat ring
sepRing = 2*PARAMETERS.ATTACKERWEAPON.F; %Mines have to be 10m apart minimum
%% Randomly place mines:
%Uncomment these next few lines of code and then comment out the 'load' 
%command a few more lines down. Hit run and see where your mines
%land. Either hit run over and over until you get a minefield that meets
%the given criteria OR manually change the positons by typing into the
%Command Window commands like:
%randomMines(1,16) = 0 %change the x position of the 16th mine
%randomMines(2,16) = -40 %change the y position of the 16th mine

%When you get a field that suits you, type into the Command Window 
% save('randomMines')
% and then uncomment the line below that says
% load('randomMines')

% randomMinesX = randi([0, 200], 1, 30)
% randomMinesY = randi([-50, 50], 1, 30)
% randomMines = [randomMinesX;randomMinesY];

%load('randomMines')
%Make this 1 to draw 10m rings around the mines, 0 to turn them off:
separationBool = 1;

%Pick your waypoints. THis format is read as [x,y] and it's easier
%to type out, but harder to plot. I transpose it right afterwards to plot
%it using the same format as the randomMines matrix.
% wptmat0 = [0, 0;
%             10, 0;
%             30, 37;
%             90,22;
%             110,10;
%             140, 5;
%             155, 30;
%             199, 10;
%             199, -10;
%             110, -15;
%             85,  -35;
%             60,  -15;
%             35,  -30;
%             0,   -10;
%             0, 0]'      

wptmat0 = p_hvu;
        
figure(600 + segment_idx);
hold on
grid on
grid minor
% axis([-10 210 -60 60])
%Draw and start and end points, put a green box around them.
% plot(startPointA(1), startPointA(2), 'gs', goalPointB(1), goalPointB(2), 'gs')
% rectangle('Position', [startPointA(1)-mineRing/2, startPointA(2)-mineRing/2, mineRing, mineRing], 'EdgeColor', 'g')
% rectangle('Position', [goalPointB(1)-mineRing/2, goalPointB(2)-mineRing/2, mineRing, mineRing], 'EdgeColor', 'g')
%Draw a black box to show the "in play" area that we can't leave:
% rectangle('Position', [0, -50, 200, 100], 'EdgeColor', 'k')
%plot the waypoints:
%plot(wptmat0, ':bs')
%plot the mines:
% randomMines = randomMines';
N_mines = PARAMETERS.N_mines;
randomMines = PARAMETERS.randomMines;
plot(randomMines(2,:), randomMines(1,:), 'r*'); hold on
plot(wptmat0(2,:), wptmat0(1,:))
%plot the circles around the mines:
for i = 1:N_mines
    %draw a circle using the rectagle function
    rectangle('Position',[randomMines(2, i)-mineRing/2, randomMines(1, i)-mineRing/2, ...
        mineRing, mineRing],'Curvature',[1,1], 'EdgeColor', 'r');
end
%plot the 10m separation rings:
%This is turned on and off in separationBool = 1.
if separationBool == 1
    for i = 1:N_mines
        %draw a circle using the rectagle function
        rectangle('Position',[randomMines(2, i)-sepRing/2, randomMines(1, i)-sepRing/2, ...
            sepRing, sepRing],'Curvature',[1,1], 'EdgeColor', 'm');
    end
end
title({['\psi_0 = ', num2str(rad2deg(psi0)), '\circ'], 'Probability of Survival: ', num2str(P_survival)});
xlabel('Y (East) [m]'); ylabel('X (North) [m]');
hold off

dist = [];
for ii = 1:Nt
    distance_2HVU_sq = (randomMines(1,1:N_mines) - p_hvu(1,ii)).^2 + (randomMines(2,1:N_mines) - p_hvu(2,ii)).^2 ;
    dist = [dist;distance_2HVU_sq];
end

[i,j] = find(dist < PARAMETERS.ATTACKERWEAPON.F^2);
j = unique(j); 
disp('Destroy the following mines')
randomMines(:,j)
removedMines = randomMines(:,j);
temp = randomMines;
temp(:,j) = [];
N_mines_new = length(temp); 
randomMines_new = temp;

if isempty(temp)
    disp('done')
else
    randomMines = randomMines_new;
    N_mines = N_mines_new;
    figure(700 +segment_idx); 
    hold on
    grid on 
    plot(randomMines(2,:), randomMines(1,:), 'r*');
    plot(wptmat0(2,:), wptmat0(1,:))
    %plot the circles around the mines:
    for i = 1:N_mines
        %draw a circle using the rectagle function
        rectangle('Position',[randomMines(2, i)-mineRing/2, randomMines(1, i)-mineRing/2, ...
            mineRing, mineRing],'Curvature',[1,1], 'EdgeColor', 'r');
    end
    %plot the 10m separation rings:
    %This is turned on and off in separationBool = 1.
    if separationBool == 1
        for i = 1:N_mines
            %draw a circle using the rectagle function
            rectangle('Position',[randomMines(2, i)-sepRing/2, randomMines(1, i)-sepRing/2, ...
                sepRing, sepRing],'Curvature',[1,1], 'EdgeColor', 'm');
        end
    end
    title({['\psi_0 = ', num2str(rad2deg(psi0)), '\circ'], ['Probability of Survival: ', num2str(P_survival)],...
           ['# of Mines Removed: ', num2str(size(removedMines,2))]});
    xlabel('Y (East) [m]'); ylabel('X (North) [m]');
    hold off
    grid minor
end
