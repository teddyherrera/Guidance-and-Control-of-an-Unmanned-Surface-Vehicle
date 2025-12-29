% ME4811        Introduction to Engineering System Dynamics and Control
%                Naval Postgraduate School, Monterey CA
%% Title:       MCM_v4
% Students:     Teddy Herrera, Joesph Young, Braden Zukowski
% Desciption:   Initializes the minefield navigation problem, provides
%                   logic for waypoint navigation, and uses different
%                   intial guesses to find a path through a mine fileld that maximizes
%                   HVU probability of survival P.
% %%----Changes from Previous Version----%%
% Functionality for finding solutions using bounded polynomials, A*, or 
% hybrid A* algorithm to generate the initital guess that computes all
% vehicle states and controls. 
% Note:
% *** Using Bezier curves ***
% Find a path through a mine field tha maximizes HVU probability of survival P.
% The code uses global data structure to pass information
%     Minimize:      J = 1 - P(tf) or -log(P(tf)), tf fixed
%
%     subject to:    dx/dt = v*r1
%                    dy/dt = v*r2
%                    dr1/dt  = -r2*r
%                    dr2/dt  = r1*r
%                    dr/dt = u1;
%                    dv/dt = u2
%                    r1^2 + r2^2 = 1
%                    where  R = [r1 -r2 0; r2 r1 0; 0 0 1]; and r1 = cos(psi); r2 = sin(psi);
%

close all; clear; clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%rng(123)
use_astar = false;
use_hybridAStar = true;
use_smoothing = true;
%%% smoothing degree for Bezier Curve %%%
n = 20;

% mine field data
N_mines =  20;
ymin = -10; ymax = 10; xmax = 100;
randomMinesX = randi([-10, 10], 1, N_mines);
randomMinesY = randi([ymin, ymax], 1, N_mines);
randomMines = [randomMinesX; randomMinesY];

data.N_mines = N_mines;
data.randomMines = randomMines;

% mine characteristics
data.ATTACKERWEAPON.lambda = 1; % firing rate
data.ATTACKERWEAPON.F = 1.5;      % lethality radius
data.ATTACKERWEAPON.sigma = 0.02; % lethality drop-off/sigma (the smaller the steeper the drop-off)
data.ATTACKERWEAPON.a = 1;        % not sure
data.MinRange = 2.0 ;

%% Note: N >= d0 + dF + 1
d0 = 2;
dF = 2;
N =  40; % order of solution

%% Problem Specifics
Nv = 1; % number of vehicles
Nx = 6; % number of states
Nu = 2; % number of controls

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Define Waypoints for Navigation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Define positions
wps_x = [-15;   % Start
          15;   % WP 2
         -15];  % Back to Start
wps_y = zeros(3,1);
% Initial heading(s) computed from the waypoints (only for the first segment)
wps_psi = atan2(diff(wps_y), diff(wps_x));
wps_psi(end+1) = wps_psi(1);  % For the first segment, use the same heading at the end as at the beginning

% Assemble waypoints: each row is [x, y, psi]
waypoints = [wps_x, wps_y, wps_psi];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Storage for the complete trajectory
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
complete_trajectory = [];
complete_velocity = [];
complete_turnrate = [];
complete_controls = [];
complete_Cx = [];
complete_DCx = [];
complete_time = [];
randomMines_removed = [];
Psi_f = 0;  % initial final heading (will be updated after first segment)
total_time = 0;

num_segments = size(waypoints,1)-1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Waypoint Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for segment_idx = 1:num_segments
    flag = [];
    x_path = [];
    y_path = [];
    psi_path = [];
    r1 = [];
    r2 = [];
    vel = [];
    turnRate = [];

    wp_start_idx = segment_idx;
    wp_end_idx = segment_idx + 1;
    
    % Extract start and end positions
    P_0 = waypoints(wp_start_idx, 1:2);
    P_f = waypoints(wp_end_idx, 1:2);
    
    % For the heading, enforce continuity: for the first segment use the
    % given waypoint heading, for subsequent segments use the final heading
    % from the previous segment.
    if segment_idx == 1
        Psi_0_segment = waypoints(wp_start_idx, 3);
    else
        Psi_0_segment = Psi_f;
    end
    % In this example, we force the segment to have the same initial and final heading.
    % You can modify this if you wish the final heading to be determined differently.
    Psi_f_desired = Psi_0_segment;
    
    % Set targets as [position, heading] for initial and final conditions
    targets = [P_0, Psi_0_segment; P_f, Psi_f_desired];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Constraints
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    rMax = 1;   % rad/s
    rdMax = 1;
    Vmax0 = 3.0;
    Vmin0 = 1.00;
    Vdmax = 1;
    Vdmin = -1;
    dmin = 0.1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Boundary Conditions
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ll = 1;
    psi0 = targets(ll,3);
    x0 = [targets(ll,1:2) cos(psi0) sin(psi0) Vmin0+1.5 0];  % initial state: [x, y, cos(psi), sin(psi), velocity, turn rate]
    psiF = targets(ll+1,3);
    xF = [targets(ll+1,1:2) cos(psiF) sin(psiF) Vmin0+1.5 0]; % final state
    
    V0 = 2.5;   % m/s  (velocity)
    VF = 2.5;   % m/s
    r0 = 0;     % initial turn rate
    rF = 0;     % final turn rate

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% final time, etc.
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    tf = norm(P_f - P_0)/V0;
    tmin = norm(P_f - P_0)/Vmax0;

    dt = 0.1;
    time = 0:dt:tf;
    time0 = time/tf;
    Nt = length(time);

    % Use A* or Hybrid A* for the initial guess
    if use_astar
        disp('Generating A* path for initial guess...');
        grid_resolution = 0.25; % meters
        astar_path = astar_path_planning_v1(P_0, P_f, randomMines, data.ATTACKERWEAPON.F, grid_resolution);

        % Extract state variables from A* path
        num_points = size(astar_path, 1);
        delta_t = tf / (num_points - 1); 

        dx = diff(astar_path(:,1));
        dy = diff(astar_path(:,2));
        psi = atan2(dy, dx);
        psi = [psi; psi(end)];

        distances = sqrt(dx.^2 + dy.^2);
        velocity = [V0; distances / delta_t]; 
        turn_rate = [0; diff(psi) / delta_t]; % assume zero initial turn rate

        r1 = cos(psi);
        r2 = sin(psi);

        states = [astar_path, r1, r2, velocity, turn_rate];

        % Convert path to Bernstein control points
        Cx_Bernstein = fitBernsteinPolynomial(states);
        Cx = elevateBernstein(Cx_Bernstein, N);

        figure; hold on;
        for i = 1:N_mines
            plot(randomMines(2,i), randomMines(1,i) ,'r*');
            mine_radius = 2 * data.ATTACKERWEAPON.F;
            rectangle('Position', [randomMines(2,i)-mine_radius/2, randomMines(1,i)-mine_radius/2, mine_radius, mine_radius], 'Curvature', [1, 1], 'EdgeColor','r');
        end
        plot(waypoints(:,2), waypoints(:,1), 'go', 'MarkerFaceColor','g');
        for i = 1:size(waypoints, 1)
            text(waypoints(i,2)+0.5, waypoints(i,1)+0.5, sprintf('WP%d', i));
        end
        plot(astar_path(:,2), astar_path(:,1), 'b--');
        xlabel('Y (East) [m]'); ylabel('X (North) [m]');
        hold off;

    elseif use_hybridAStar
        disp('Generating Hybrid A* path for initial guess...');
        grid_resolution = 0.25; % meters
        astar_path = HybridAstar_path_planning_v1_1([P_0, psi0], ...
            [P_f, atan2(P_f(2) - P_0(2), P_f(1) - P_0(1))], ...
            randomMines, data.ATTACKERWEAPON.F, grid_resolution, rMax, Vmin0);

        num_points = size(astar_path, 1);
        delta_t = tf / (num_points - 1); 

        dx = diff(astar_path(:,1));
        dy = diff(astar_path(:,2));
        psi = astar_path(:,3);

        distances = sqrt(dx.^2 + dy.^2);
        velocity = [V0; distances / delta_t]; 
        turn_rate = [0; diff(psi) / delta_t];

        r1 = cos(psi);
        r2 = sin(psi);

        states = [astar_path(:,1), astar_path(:,2), r1, r2, velocity, turn_rate];

        Cx_Bernstein = fitBernsteinPolynomial(states);
        Cx = elevateBernstein(Cx_Bernstein, N);

        figure; hold on;
        for i = 1:N_mines
            plot(randomMines(2,i), randomMines(1,i), 'r*');
            mine_radius = 2 * data.ATTACKERWEAPON.F;
            rectangle('Position', [randomMines(2,i)-mine_radius/2, randomMines(1,i)-mine_radius/2, mine_radius, mine_radius], 'Curvature', [1, 1], 'EdgeColor','r');
        end
        plot(waypoints(:,2), waypoints(:,1), 'go', 'MarkerFaceColor','g');
        for i = 1:size(waypoints, 1)
            text(waypoints(i,2)+0.5, waypoints(i,1)+0.5, sprintf('WP%d', i));
        end
        plot(astar_path(:,2), astar_path(:,1), 'b--');
        xlabel('Y (East) [m]'); ylabel('X (North) [m]');
        hold off;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Bezier Setup
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Dm = Diff_elev(N,1);            
    DmSq = Dm * Dm;                    
    BN = bernsteinMatrix(N, time0);
    BN1 = bernsteinMatrix(3*N, time0);
    BNa2b = bernsteinMatrix_a2b(N, time);

    DU_total = [];
    for jj = 1:Nv
        k = max(max(targets));
        DU = diag([k; k; 1; 1; Vmax0; 1]);  
        DU_total = blkdiag(DU_total, DU);
        x0(jj,:) = x0(jj,:) / DU;
        xF(jj,:) = xF(jj,:) / DU;
    end

    V0 = V0 / Vmax0;
    VF = VF / Vmax0;
    Vmin = Vmin0 / Vmax0;
    Vmax = Vmax0 / Vmax0;
    dmin = dmin / k; 

    tic
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Global variable setup for optimization
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    data.N = N; data.Dm = Dm; data.DmSq = DmSq; data.BN = BN; data.BNa2b = BNa2b; 
    data.dt = dt; data.Nv = Nv; data.Nx = Nx; data.Nu = Nu; data.Nt = Nt;
    data.time = time0; data.rMax = rMax;
    data.DU_total = DU_total;
    data.tf = tf;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Generate initial guess for trajectory
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if use_astar || use_hybridAStar
        Cx([1 end],1:2) = [P_0; P_f];
        Cx = Cx / k;
        DCx = Dm' * Cx; 
        Cu = DCx(:, [6 5]); 
    else
        %%% Initialization using a 5th order polynomial (Oleg's approach)
        p0 = x0(1:2); pdot0 = V0 * x0(3:4); pdd0 = zeros(1,2);
        pF = xF(1:2); pdotF = VF * xF(3:4); pddF = zeros(1,2);
        V0dot = 0; VFdot = 0; V0ddot = 0; VFddot = 0;

        DmDF = Diff_elev(5,1);
        CoeffMatrix = [bernsteinMatrix(5,0); bernsteinMatrix(5,0)*DmDF'; bernsteinMatrix(5,0)*DmDF'*DmDF';...
            bernsteinMatrix(5,1)*DmDF'*DmDF'; bernsteinMatrix(5,1)*DmDF'; bernsteinMatrix(5,1)];

        Cp = CoeffMatrix \ [p0; pdot0; pdd0; pddF; pdotF; pF];
        Cvel = CoeffMatrix \ [V0; V0dot; V0ddot; VFddot; VFdot; VF];

        for j = 5:N-1
            j = j + 1;
            D_elev = deg_elev(j);
            Cp = Cp' * D_elev{j-1};
            Cp = Cp';
            Cvel = Cvel' * D_elev{j-1};
            Cvel = Cvel';
        end
        Cv = Dm' * Cp;
        Cv_x = Cv(:,1); Cv_y = Cv(:,2);
        Cr = Cv ./ sqrt(sum(Cv.^2,2));
        Cacc = Dm' * Cv;
        Cacc_x = Cacc(:,1); Cacc_y = Cacc(:,2);
        CturnRate = (Cacc_y .* Cv_x - Cacc_x .* Cv_y);
        CturnRate = CturnRate ./ (sum(Cv.^2,2));
        Cu1 = Dm' * CturnRate;
        Cu2 = Dm' * Cvel;
        Cu = [Cu1 Cu2];
        Cx = [Cp Cr Cvel CturnRate];
    end

    % Plot initial guess for this segment
    for jj = 1:Nv
        figure(100 + 10*segment_idx)
        P = k * Cx(:,1:2);
        plot(P(:,2), P(:,1)); grid on;
        title(['\psi_0 = ', num2str(rad2deg(psi0)),'\circ']);
        xlabel('Y (East) [m]'); ylabel('X (North) [m]'); axis equal;

        figure(200 + 10*segment_idx)
        u = BNa2b * Cu;
        plot(time0, u); grid on;
        title(['\psi_0 = ', num2str(rad2deg(psi0)), '\circ'])
        legend('turn rate', 'acceleration');
    end

    data.DU = DU;
    data.tf = tf;

    if segment_idx == 1
        Aeq = zeros(8, (N+1)*(Nx+Nu)+1);
        Aeq(1,1) = 1;           % x0
        Aeq(2, N+2) = 1;         % y0
        Aeq(3, 4*(N+1)+1) = 1;   % V0
        Aeq(4, 5*(N+1)+1) = 1;   % r0
        Aeq(5, (N+1)) = 1;       % xf
        Aeq(6, 2*(N+1)) = 1;     % yf
        Aeq(7, 5*(N+1)) = 1;     % VF
        Aeq(8, 6*(N+1)) = 1;     % rF
        beq = [x0(1); x0(2); x0(5); x0(6); xF(1); xF(2); xF(5); xF(6)];
    else
        Aeq = zeros(10, (N+1)*(Nx+Nu)+1);
        Aeq(1,1) = 1;         % x0
        Aeq(2, N+2) = 1;       % y0
        Aeq(3, 2*(N+1)+1) = 1; % r1 at start
        Aeq(4, 3*(N+1)+1) = 1; % r2 at start
        Aeq(5, 4*(N+1)+1) = 1; % V0
        Aeq(6, 5*(N+1)+1) = 1; % r0
        Aeq(7, (N+1)) = 1;     % xf
        Aeq(8, 2*(N+1)) = 1;   % yf
        Aeq(9, 5*(N+1)) = 1;   % VF
        Aeq(10, 6*(N+1)) = 1;  % rF
        beq = [x0(1:6)'; xF(1:2)'; xF(5:6)'];
    end
if segment_idx > 1
        % Compute indices based on global optimization vector layout
        N_control = (N + 1) * Nx;  % Number of control points per segment
        total_vars = (N+1)*(Nx + Nu) + 1; % State, control, tf per segment
        % Previous segment final Cx index (last state control point)
        idx_prev = (N) * Nx + (1:Nx);    % Last Cx of previous segment (segment_idx - 1)
        % Current segment first Cx index (first state control point)
        idx_next = (0) * Nx + (1:Nx);    % First Cx of this segment (segment_idx)
        % Initialize new Aeq rows for continuity (state only here)
        Aeq_cont = zeros(Nx, total_vars);
        for k = 1:Nx
            Aeq_cont(k, idx_prev(k)) = -1;        % Enforce Cx_prev(N+1,:) = Cx_next(1,:)
            Aeq_cont(k, idx_next(k)) = 1;
        end
        beq_cont = zeros(Nx, 1);
        % Append the continuity constraint
        Aeq = [Aeq; Aeq_cont];
        beq = [beq; beq_cont];
        %% Optional: Derivative Continuity
        % (Insert this if you want derivative continuity right after)
        idx_prev_end   = ((N-1) * Nx + (1:Nx)); % C_{N-1}
        idx_prev_last  = ((N) * Nx + (1:Nx));   % C_N
        idx_next_first = (0 * Nx + (1:Nx));     % C_0 of next
        idx_next_second= (1 * Nx + (1:Nx));     % C_1 of next
        Aeq_deriv = zeros(Nx, total_vars);
        for k = 1:Nx
            Aeq_deriv(k, idx_prev_last(k))   =  1;
            Aeq_deriv(k, idx_prev_end(k))    = -1;
            Aeq_deriv(k, idx_next_first(k))  = -1;
            Aeq_deriv(k, idx_next_second(k)) =  1;
        end
        beq_deriv = zeros(Nx, 1);
        Aeq = [Aeq; Aeq_deriv];
        beq = [beq; beq_deriv];
end
    Cx = reshape(Cx, [Nx*(N+1),1]);
    Cu = reshape(Cu, [Nu*(N+1),1]);
    X = [Cx; Cu; tf];
    difference = Aeq * X - beq;

    x_lb = [-ones((Nx-2)*(N+1),1)*inf; ones((N+1),1)*Vmin; -ones((N+1),1)*rMax];
    x_ub = [ones((Nx-2)*(N+1),1)*inf; ones((N+1),1)*Vmax; ones((N+1),1)*rMax];

    u_lb = [-ones((N+1),1)*rdMax; ones((N+1),1)*Vdmin];
    u_ub = [ones((N+1),1)*rdMax; ones((N+1),1)*Vdmax];

    t_lb = tmin;
    t_ub = inf;
    lb = [x_lb(:); u_lb; t_lb];
    ub = [x_ub(:); u_ub; t_ub];

    A = zeros(2, (N+1)*(Nx+Nu)+1);
    A(1, N+2:2*(N+1)) = 1;   % upper bound on y
    A(2, N+2:2*(N+1)) = -1;  % lower bound on y
    b = [ymax/k; ymin/k];

    options = optimoptions(@fmincon,'Algorithm','interior-point',...
        'MaxFunctionEvaluations',1e6,...
        'ConstraintTolerance',1e-6,...
        'StepTolerance',1e-6,...
        "EnableFeasibilityMode",true,...
        'Display','iter','MaxIterations',20);

    [xOut,Jout,exitflag,output] = fmincon(@(x)costFunc(x,data), X, A, b, Aeq, beq, lb, ub, @(x)nonlcon(x,data), options);
    flag = [flag; exitflag];
    tf = xOut(end);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Plot and Process Results for this Segment
    [Cx, DCx, Cu] = getTrajectories(xOut, data);
    Cx  = BNa2b * Cx * DU_total;
    DCx = BNa2b * DCx * DU_total / tf;
    Cu  = BNa2b * Cu;
    Cp = Cx(:,1:2);
    analysis_v2_1

    for i = 1:Nv
        x = Cx(:, Nx*(i-1)+1); 
        y = Cx(:, Nx*(i-1)+2);
        r1 = Cx(:, Nx*(i-1)+3);  
        r2 = Cx(:, Nx*(i-1)+4);
        v = Cx(:, Nx*(i-1)+5); 
        r = Cx(:, Nx*(i-1)+6);
        rdot = Cu(:, Nu*(i-1)+1); 
        acc = Cu(:, Nu*(i-1)+2);

        figure(300 + 10*segment_idx);
        set(gcf,'Position',[112 123 560 975]);
        subplot(6,1,1:4); hold on;
        plot(y, x); grid on;
        title('Path'); ylabel('X (North) [m]');
        xlabel('Y (East) [m]');
        subplot(6,1,5); hold on;
        plot(time, x); grid on;
        ylabel('X (North) [m]');
        subplot(6,1,6); hold on;
        plot(time, y); grid on;
        ylabel('Y (East) [m]'); xlabel('Time [s]');
        sgtitle(['\psi_0 = ', num2str(rad2deg(Psi_0_segment)), '\circ'])

        figure(400 + 10*segment_idx);
        subplot(2,1,1)
        plot(time, v); legend('speed'); hold on;
        subplot(2,1,2)
        plot(time, r); legend('turn rate'); hold on;
        xlabel('time');
        sgtitle(['\psi_0 = ', num2str(rad2deg(Psi_0_segment)), '\circ'])

        figure(500 + 10*segment_idx);
        subplot(2,1,1)
        plot(time, acc'); legend('acceleration'); hold on;
        subplot(2,1,2)
        plot(time, rdot'); legend('turn acceleration'); hold on;
        xlabel('time');
        sgtitle(['\psi_0 = ', num2str(rad2deg(Psi_0_segment)), '\circ'])
    end

    % Update final heading from this segment to be used in the next segment
    x_end = Cx(:,1); y_end = Cx(:,2);
    r1_end = Cx(:,3); r2_end = Cx(:,4);
    Psi_f = atan2(r2_end(end), r1_end(end));

    fprintf('\n%d/%d WP Segment Optimization complete. Best initial heading: %.2f rad (%.1f°)\n',...
        segment_idx, num_segments, Psi_f, rad2deg(Psi_f));

    % Update minefield scenario for next segment (if applicable)

    MinimizedMinefield = randomMines;  % or comment out this block if no update is needed
    N_mines = length(MinimizedMinefield); 
    data.N_mines = N_mines; 
    randomMines = MinimizedMinefield; 
    data.randomMines = MinimizedMinefield;

    segment_time = linspace(0, tf, Nt);
    segment_time_shifted = segment_time + total_time;
    total_time = total_time + tf;

    complete_trajectory = [complete_trajectory(1:end-1,:); [x, y]];
    complete_velocity = [complete_velocity(1:end-1,:); v];
    complete_turnrate = [complete_turnrate(1:end-1,:); r];
    complete_controls = [complete_controls(1:end-1,:); [rdot, acc]];
    complete_time = [complete_time(1:end-1,:); segment_time_shifted'];
    if segment_idx == 1
        complete_Cx = Cx;
        complete_DCx = DCx;
    else
        complete_Cx = [complete_Cx(1:end-1,:); Cx];
        complete_DCx = [complete_DCx(1:end-1,:); DCx];
    end
end
if use_smoothing
    %%%%% Smoothing Finalized Trajectory for Fixing Midpoint %%%%%
    M = size(complete_Cx, 1);
    
    % Create a normalized time vector for your trajectory points.
    t_global = linspace(0,1,M)';  
    
    % Compute the Bernstein matrix and Control Points.
    B_global = bernsteinMatrix(n, t_global);
    control_points = B_global \ complete_Cx;
    
    % Computing Discretized States
    t_fine = linspace(0,1,size(complete_Cx,1))';
    B_fine = bernsteinMatrix(n, t_fine);
    complete_Cx = B_fine * control_points;
    control_points_deriv = B_global \ complete_DCx;
    complete_DCx = B_fine * control_points_deriv;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot Summary Figure
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1000); hold on;
for i = 1:N_mines
    plot(randomMines(2,i), randomMines(1,i), 'r*');
    mine_radius = 2 * data.ATTACKERWEAPON.F;
    rectangle('Position', [randomMines(2,i)-mine_radius/2, randomMines(1,i)-mine_radius/2, mine_radius, mine_radius], 'Curvature', [1,1], 'EdgeColor', 'r');
end
plot(waypoints(:,2), waypoints(:,1), 'go', 'MarkerFaceColor', 'g');
for i = 1:size(waypoints, 1)
    text(waypoints(i,2)+1.0, waypoints(i,1)+1.0, sprintf('%d', i), 'FontSize', 10);
end
plot(complete_Cx(:,2), complete_Cx(:,1), 'b-', 'LineWidth', 2);
title('Complete Trajectory');
xlabel('Y (East) [m]');
ylabel('X (North) [m]');
grid on;
hold off;

figure(1001);
subplot(311);
plot(complete_time, complete_velocity, 'b-', 'LineWidth', 1.5);
grid on;
title('Velocity Profile');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
subplot(312);
plot(complete_time, complete_turnrate, 'g-', 'LineWidth', 1.5);
grid on;
title('Turn Rate');
xlabel('Time [s]');
ylabel('Turn Rate [rad/s]');
subplot(313);
plot(complete_time, complete_controls(:,1), 'b-', 'LineWidth', 1);
hold on;
plot(complete_time, complete_controls(:,2), 'r-', 'LineWidth', 1);
grid on;
title('Control Inputs');
xlabel('Time [s]');
ylabel('Control');
legend('Turn Rate Control', 'Acceleration Control', 'Location', 'best');

disp('Destroy the following mines [x ; y]');
disp(randomMines_removed);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Supporting Functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function C_init = fitBernsteinPolynomial(states)
    n_points = size(states, 1);
    t_normalized = linspace(0, 1, n_points)';
    BM = bernsteinMatrix(5, t_normalized);
    C_init = BM \ states;
end

function CN = elevateBernstein(C_bernstein, N)
    for j = size(C_bernstein,1)-1:N-1
        j = j + 1;
        D_elev = deg_elev(j);
        C_bernstein = C_bernstein' * D_elev{j-1};
        C_bernstein = C_bernstein';
    end
    CN = C_bernstein;
end

function [c, ceq] = nonlcon(x, data)
    N = data.N; Nx = data.Nx; DU_total = data.DU_total;
    Nv = data.Nv; BNa2b = data.BNa2b; Nt = data.Nt;
    [Cx, DCx, Cu] = getTrajectories(x, data);
    tf = data.tf;
    fxu = f_x_u(Cx, Cu, data);
    ceq = DCx - tf * fxu;
    ceq = reshape(ceq, (N+1) * Nx * Nv, 1);
    
    PARAMETERS = data;
    N = PARAMETERS.N;
    BNa2b = PARAMETERS.BNa2b;
    Nt = PARAMETERS.Nt;
    dt = PARAMETERS.dt;
    N_mines = PARAMETERS.N_mines;
    randomMines = PARAMETERS.randomMines;
    DU_total = data.DU_total;
    Cp = Cx(:,1:2);
    p_hvu = (BNa2b * Cp)';
    
    c = [];
    for ii = 1:Nt
        distance_2HVU_sq = 1 - ((randomMines(1,1:N_mines) - p_hvu(1,ii)).^2 + (randomMines(2,1:N_mines) - p_hvu(2,ii)).^2) / (data.MinRange^2);
        c = [c; distance_2HVU_sq'];
    end
end

function [Cx, DCx, Cu] = getTrajectories(X, data)
    N = data.N; Nx = data.Nx; Nu = data.Nu; Dm = data.Dm; Nv = data.Nv;
    Cx = X(1:Nv * Nx * (N+1), 1);
    Cx = reshape(Cx, [N+1, Nx * Nv]);
    Cu = X(Nv * Nx * (N+1) + 1 : end-1);
    Cu = reshape(Cu, [N+1, Nv * Nu]);
    DCx = Dm' * Cx;
end

function fxu = f_x_u(Cx, Cu, data)
    N = data.N; Nx = data.Nx; Nu = data.Nu; Dm = data.Dm; Nv = data.Nv;
    DU = data.DU; DU_total = data.DU_total;
    Cx = Cx * DU_total;
    fxu = [];
    for i = 1:Nv
        r1 = Cx(:, Nx*(i-1)+3); 
        r2 = Cx(:, Nx*(i-1)+4); 
        v  = Cx(:, Nx*(i-1)+5); 
        r  = Cx(:, Nx*(i-1)+6);
        u2 = Cu(:, Nu*(i-1)+2); 
        u1 = Cu(:, Nu*(i-1)+1);
        Mag = sqrt(r1.^2 + r2.^2);
        r1 = (r1 + eps) ./ (Mag + eps);
        r2 = (r2 + eps) ./ (Mag + eps);
        r1 = sqrt(1 - r2.^2);
        fxu0 = [v .* r1, v .* r2, -r2 .* r, r1 .* r, u1, u2] / DU;
        fxu = [fxu, fxu0];
    end
end

function J = costFunc(X, data)
    PARAMETERS = data;
    N = PARAMETERS.N;
    BNa2b = PARAMETERS.BNa2b;
    Nt = PARAMETERS.Nt;
    dt = PARAMETERS.dt;
    N_mines = PARAMETERS.N_mines;
    randomMines = PARAMETERS.randomMines;
    DU_total = data.DU_total;
    MinRange = data.MinRange;
    
    Nx = data.Nx; Nv = data.Nv;
    Cx = X(1:Nv * Nx * (N+1), 1);
    Cx = reshape(Cx, [N+1, Nx * Nv]);
    Cx = Cx * DU_total;
    Cp = Cx(:,1:2);
    p_hvu = (BNa2b * Cp)';
    
    tf = X(end);
    P = 1;
    mysum = 0;
    for ii = 1:Nt
        distance_2HVU_sq = (randomMines(1,1:N_mines) - p_hvu(1,ii)).^2 + (randomMines(2,1:N_mines) - p_hvu(2,ii)).^2;
        ra2hvu = PARAMETERS.ATTACKERWEAPON.lambda * normcdf((PARAMETERS.ATTACKERWEAPON.F - PARAMETERS.ATTACKERWEAPON.a * distance_2HVU_sq) / PARAMETERS.ATTACKERWEAPON.sigma, 0, 1);
        deltaP = prod(1 - ra2hvu * dt);
        P = P * deltaP;
        mysum = mysum + sum(1./(distance_2HVU_sq) - MinRange^2)^2;
    end
    J = -log(P);
end

function Dm = Diff(N, tf)
    Dm = -[N/tf * eye(N); zeros(1, N)] + [zeros(1, N); N/tf * eye(N)];
end

function Telev = deg_elev(N)
    if N < 5
        error('ERROR: The approximation order should be at least 5');
    end
    for i = 1:N
        Telev{i} = zeros(i+2, i+1);
        for j = 1:i+1
            Telev{i}(j, j) = i+1 - (j-1);
            Telev{i}(j+1, j) = 1 + (j-1);
        end
        Telev{i} = 1/(i+1) * Telev{i}';
    end
end

function Dm = Diff_elev(N, tf)
    Dm = Diff(N, tf);
    Telev = deg_elev(N);
    Dm = Dm * Telev{N-1};
end

function Prod_T = Prod_Matrix(N)
    T = zeros(2*N+1, (N+1)^2);
    for j = 0:2*N
        for i = max(0, j-N):min(N, j)
            if N >= i && N >= j-i && 2*N >= j && j-i >= 0
                T(j+1, N*i+j+1) = nchoosek_mod(N, i) * nchoosek_mod(N, j-i) / nchoosek_mod(2*N, j);
            end
        end
    end
    Prod_T = T;
end

function out = nchoosek_mod(n, k)
    out = 1;
    for i = 1:k
        out = out * (n - (k - i));
        out = out / i;
    end
end

function b = bernsteinMatrix_a2b(n, t, tmin, tmax)
    if length(t) > 1
        tmax = t(end);
        tmin = t(1);
    end
    if ~(isscalar(n) && isreal(n) && n == round(n) && n >= 0)
        error('Expecting a nonnegative integer for n');
    end
    if isempty(t)
        b = ones(0, n+1);
        return;
    end
    if ~isvector(t)
        error('Expecting t to be a vector');
    end
    t = t';
    B = ones(n+1, 1);
    for j = 1:ceil(n/2)
        B(j+1) = B(j) * (n+1-j) / j;
        B(n+1-j) = B(j+1);
    end
    T = ones(length(t), n+1);
    TT = ones(length(t), n+1);
    tp = t - tmin;
    ttp = tmax - t;
    for j = 1:n
        T(:, j+1) = tp .* T(:, j);
        T(:, j) = T(:, j) .* B(j);
        TT(:, n+1-j) = ttp .* TT(:, n+2-j);
    end
    b = T .* TT ./ ((tmax - tmin)^n);
end

function b = bernsteinMatrix(n, t)
    % Standard Bernstein matrix for time vector t in [0,1]
    t = t(:);
    b = zeros(length(t), n+1);
    for j = 0:n
        b(:, j+1) = nchoosek(n, j) * (t.^j) .* ((1-t).^(n-j));
    end
end