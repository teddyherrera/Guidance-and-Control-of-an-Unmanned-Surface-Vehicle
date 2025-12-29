function path = astar_path_planning_v1(start_pos, goal_pos, mines, mine_radius, grid_resolution)

% set the map boundaries
xmin = min(start_pos(1), goal_pos(1));
xmax = max(start_pos(1), goal_pos(1));
ymin = min(start_pos(2), goal_pos(2))-15;
ymax = max(start_pos(2), goal_pos(2))+15;

% Create occupancy map
map = binaryOccupancyMap(xmax - xmin, ymax-ymin, 1/grid_resolution);
map.GridLocationInWorld = [ymin, xmin];

for i = 1:size(mines, 2)
    mine_x = mines(1,i);
    mine_y = mines(2,i);
    
    setOccupancy(map, [mine_x mine_y], 1);
end

inflate(map, mine_radius); 

figure; 
show(map);

% Create A* planner
planner = plannerAStarGrid(map);


try
    path = plan(planner, start_pos, goal_pos, 'world');
    figure;
    show(planner);
catch
    warning('A* Path Planning failed. Using direct line.');
    path = [start_pos; goal_pos];
end
end