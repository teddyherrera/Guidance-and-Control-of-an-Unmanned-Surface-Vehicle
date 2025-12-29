% ME4811        Introduction to Engineering System Dynamics and Control
%                Naval Postgraduate School, Monterey CA
%% Title:       plot_traversed_trajectory
% Students:     Teddy Herrera, Joesph Young, Braden Zukowski
% Desciption:   Plots traversed trajectory from simulink model.

figure(1003); clf; hold on;
for i = 1:N_mines
    plot(randomMines(2,i), randomMines(1,i), 'r*');
    mine_radius = 2 * data.ATTACKERWEAPON.F;
    rectangle('Position', [randomMines(2,i)-mine_radius/2, randomMines(1,i)-mine_radius/2, mine_radius, mine_radius], 'Curvature', [1,1], 'EdgeColor', 'r');
end
plot(waypoints(:,2), waypoints(:,1), 'go', 'MarkerFaceColor', 'm');
for i = 1:size(waypoints, 1)
    text(waypoints(i,2)+1.0, waypoints(i,1)+1.0, sprintf('%d', i), 'FontSize', 10);
end
plot(complete_Cx(:,2), complete_Cx(:,1), 'b-', 'LineWidth', 2);
title('Complete Trajectory');
xlabel('Y (East) [m]');
ylabel('X (North) [m]');
grid on;
hold off;
hold on;

% Plot the actual traversed trajectory in green
hGreen = plot(simout.Data(:,2), simout.Data(:,1), 'g', 'LineWidth', 2);

% Plot Blank handles for the legend
hBlue_blank = plot(nan, nan, 'b', 'LineWidth', 2);
hGreen_blank = plot(nan, nan, 'g', 'LineWidth', 2);
title('Trajectory Comparison: Optimal vs Actual');
legend([hBlue_blank, hGreen_blank], ...
    {'Optimally Generated Trajectory', 'Actually Traversed Trajectory'}, ...
    'Location', 'best');