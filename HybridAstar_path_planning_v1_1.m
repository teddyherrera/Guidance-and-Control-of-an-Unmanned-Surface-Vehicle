function path = HybridAstar_path_planning_v1_1(start_pos, goal_pos, mines, mine_radius, grid_resolution, rMax, vMin)

% set the map boundaries
xmin = min(start_pos(1), goal_pos(1))-2;
xmax = max(start_pos(1), goal_pos(1))+2;
ymin = min(start_pos(2), goal_pos(2))-15;
ymax = max(start_pos(2), goal_pos(2))+15;

% Create occupancy map
map = binaryOccupancyMap(xmax - xmin, ymax-ymin, 1/grid_resolution);
map.GridLocationInWorld = [xmin, ymin];

for i = 1:size(mines, 2)
    mine_x = mines(1,i);
    mine_y = mines(2,i);
    
    setOccupancy(map, [mine_x mine_y], 1);
end

% Assign map to state validator
ss = stateSpaceSE2;
ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
sv = validatorOccupancyMap(ss);
sv.ValidationDistance = 0.1; 
sv.Map = map; 

inflate(map, mine_radius); 

figure; 
show(map); axis tight


% Create A* planner

% Motion primitive length cannot exceed 1/4 the length of the circumference
% of a circle based on the 'MinTurningRadius'.
hvuTurnRadius =  vMin / rMax;
motion_Primitive = (2 * pi * hvuTurnRadius ) / 10;

planner = plannerHybridAStar(sv,...
                             MinTurningRadius=hvuTurnRadius,...
                             MotionPrimitiveLength=motion_Primitive,...
                             ReverseCost=1e6);

try
    pathObj = plan(planner, start_pos, goal_pos,"SearchMode","exhaustive");
    path = [pathObj.States(:,1), pathObj.States(:,2), pathObj.States(:,3)]; % [x, y, psi] 
    figure;
    show(planner); axis tight
catch
    warning('A* Path Planning failed. Using direct line.');
    path = [start_pos; goal_pos];
end
end