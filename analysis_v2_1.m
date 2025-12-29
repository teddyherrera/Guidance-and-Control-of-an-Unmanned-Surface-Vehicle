% ME4811        Introduction to Engineering System Dynamics and Control
%                Naval Postgraduate School, Monterey CA
%% Title:       Analysis_v2_1
% Students:     Teddy Herrera, Joesph Young, Braden Zukowski               
% Desciption:   This script computes probability of survival of an USV traveling through
%                   a mine field. Attrition rate of each mine is assumed to be circular,
%                   modeled using Poisson cumulative prob. distribution function. 
% Inputs: N, Cp and tf - order of the Bernstein pol.,  the control
% coefficients and total time of travel.
% Output: USV probability of survival and a plot of its trajectory through a random mine
% field.
% Note: Figure numbers changed to accomodate multiple headings/waypoints

PARAMETERS = data;
PARAMETERS.N = N; 
PARAMETERS.BNa2b = BNa2b; 
PARAMETERS.tf = tf; 
PARAMETERS.time = time; 
PARAMETERS.Nt = Nt; 
PARAMETERS.dt = dt; 

p_hvu = Cp';
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% randomMinesX = randi([0, 200], 1, N_mines);
% randomMinesY = randi([-50, 50], 1, N_mines);
% randomMines = [randomMinesX;randomMinesY];

PARAMETERS.N_mines = N_mines; 
PARAMETERS.randomMines = randomMines; 


% mine characteristics 

% PARAMETERS.ATTACKERWEAPON.lambda = 2; % firing rate
% PARAMETERS.ATTACKERWEAPON.F = 5;      % lethality radius
% PARAMETERS.ATTACKERWEAPON.sigma = .02; % lethality drop-off 
% PARAMETERS.ATTACKERWEAPON.a = 1; % not sure
% PARAMETERS.ATTACKERWEAPON.range = 5; % ignore the mine beyond this range

J = cost(Cp,PARAMETERS);
P_survival = J;
MinefieldAndWaypointPlotter_v2_1

function J = cost(x,PARAMETERS)

% compute probability of HVU survival P
N = PARAMETERS.N; 
BNa2b = PARAMETERS.BNa2b; 
Nt = PARAMETERS.Nt; 
dt = PARAMETERS.dt; 
N_mines = PARAMETERS.N_mines; 
randomMines = PARAMETERS.randomMines; 

p_hvu = x';
% p_hvu = (BNa2b*Cp)';

P = 1; % initial probability of HVU survival
for ii = 1:Nt
    distance_2HVU_sq = (randomMines(1,1:N_mines) - p_hvu(1,ii)).^2 + (randomMines(2,1:N_mines) - p_hvu(2,ii)).^2 ;
    ra2hvu = PARAMETERS.ATTACKERWEAPON.lambda*normcdf((PARAMETERS.ATTACKERWEAPON.F - PARAMETERS.ATTACKERWEAPON.a*distance_2HVU_sq)/PARAMETERS.ATTACKERWEAPON.sigma,0,1);
    deltaP = prod(1-ra2hvu.*dt);
    P = P.*deltaP;
end

disp("HVU Probability of Survival:"), P

% tic
% P = 1;
% for ii = 1:Nt
%     for jj = 1:N_mines
%         dx = abs(randomMines(1,jj) - p_hvu(1,ii));
%         if dx <= PARAMETERS.ATTACKERWEAPON.range
%             dy = abs(randomMines(2,jj) - p_hvu(2,ii));
%             if dy <= PARAMETERS.ATTACKERWEAPON.range
%                     if dx^2 + dy^2 <= PARAMETERS.ATTACKERWEAPON.range^2
%                         distance_2HVU_sq = (randomMines(1,jj) - p_hvu(1,ii)).^2 + (randomMines(2,jj) - p_hvu(2,ii)).^2 ;
%                         ra2hvu = PARAMETERS.ATTACKERWEAPON.lambda*normcdf((PARAMETERS.ATTACKERWEAPON.F - PARAMETERS.ATTACKERWEAPON.a*distance_2HVU_sq)/PARAMETERS.ATTACKERWEAPON.sigma,0,1);
%                         deltaP = prod(1-ra2hvu.*dt);
%                         P = P.*deltaP;
%                     end
%             end
%         end
%         
%     end
% end

J = P;
end



function b = bernsteinMatrix_a2b(n, t , tmin, tmax)

% This is like bernsteinMatrix, but it works for any time interval, i.e.
% from [a,b] rather than [0,1]
% if time t at the input is a row vector, you do not need to provide tmin
% and tmax
% if t is a scalar, you need to provide tmin and tmax

if length(t) > 1
    tmax = t(length(t));
    tmin = t(1);
end


if ~(isscalar(n) && isreal(n) && n == round(n) && n >= 0)
   error(message('symbolic:sym:bernsteinMatrix:ExpectingNonNegativeInteger1'));
end
if isempty(t) 
   b = ones(0, n+1);
   return
end
if ~isvector(t)
   error(message('symbolic:sym:bernsteinMatrix:ExpectingVector2'));
end

% compute B(j+1) = binomial(n, j), j = 0..n
t = t';
B = ones(n+1, 1); 
for j = 1:ceil(n/2)
  B(j+1) = B(j)*(n+1-j)/j;
  B(n+1-j) = B(j+1);
end
% compute T(i, j) = t(i)^j, TT(i, j) = (1-t(i))^(n-j)
T = ones(length(t), n+1);
TT = ones(length(t), n+1);
% turn a row t into a column t so that 
% we can do diag(t)*T(:,j) by t.*T(:,j)
tp = reshape(t, length(t), 1)-tmin;
ttp = tmax - t;
for j = 1:n 
  T(:, j+1) = tp .* T(:, j); 
  T(:, j) = T(:, j) .* B(j);
 TT(:,n+1-j) = ttp .* TT(:,n+2-j);
end


b =  T.*TT./((tmax-tmin)^n);
end
% 
%     if deltaP < 0
%         deltaP = 0;
%     elseif deltaP > 1
%         deltaP = 1; 
%     end
%     P = P.*(1-(1-prod(1-ra2hvu.*dt)));

% time of travel
% tf = 60;
% dt = 0.1;
% Nt = tf/dt; 
% % time increment
% 
% 
% time = 0:dt:tf-dt;
% 
% 
% % test trajectory
% p_hvu = P_0 + V_HVU*[cos(Psi_d); sin(Psi_d)].*time;
