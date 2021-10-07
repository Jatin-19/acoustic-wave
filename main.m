close all;
clear all;
clc;          

% Note: Above three commands close all previous windows, clear work space and command window

tic

N=6; % number of robots


%% Initialize 

% first robot
x(1)     =  -10;           % x-position
y(1)     =  -2;            % y-position
theta(1) =   0*pi/180;     % velocity direction (converted from degrees to radian)
v(1) = 1.0;                % speed


% second robot
x(2)     =  4;
y(2)     = -2;
theta(2) =  30*pi/180;
v(2) = 1.5;

% 3rd robot
x(3)     =  -10;           % x-position
y(3)     =  10;            % y-position
theta(3) =   45*pi/180;     % velocity direction (converted from degrees to radian)
v(3) = 2.0;                % speed


% 4th robot
x(4)     =  2;
y(4)     = 5;
theta(4) = 60*pi/180;
v(4) = 2.5;

% 5th robot
x(5)     =  0;           % x-position
y(5)     =  -5;            % y-position
theta(5) =   75*pi/180;     % velocity direction (converted from degrees to radian)
v(5) = 3.0;                % speed


% 6th robot
x(6)     =  5;
y(6)     = -7;
theta(6) =  90*pi/180;
v(6) = 3.5;



% Note: In MATLAB, it is recommended to do calculations in matrix
% notations. Here, we form row matrices x, y, and theta for x-position, y-position and velocity directions, respectively.
% The components of these vectors corrspond to indivial robots. 
% For instance x(1) and x(2) above are the first and second components of
% the row matrix x = [x(1) x(2)], and denotes the x-positions of the first and
% second robots, respectively. Similarly, it is true for vectors y = [y(1)
% y(2)] and theta = [theta(1) theta(2). 

%% controller gain

K = 0.01;

%% simulation time

T  = 100;         % total time
dt = 0.01;        % step size


%% store variables for offline plot
iStep = 1;
xStore  = zeros(round(T/dt),N);
yStore  = zeros(round(T/dt),N);
thetaStore = zeros(round(T/dt),N);
uStore  = zeros(round(T/dt),N);
time = zeros(round(T/dt),1);

%% Control Loop

for t = 0:dt:T
        
%% control law
    

    %partA
    %% u(1) = K*(theta(6) + theta(5) + theta(4) + theta(3) + theta(2) - 5 *theta(1)); % control law for the first robot
    %% u(2) = K*(theta(6) + theta(5) + theta(4) + theta(3) + theta(1) - 5 *theta(2)); % control law for the second robot 
    %% u(3) = K*(theta(6) + theta(5) + theta(4) + theta(2) + theta(1) - 5 *theta(3)); % control law for the second robot 
    %% u(4) = K*(theta(6) + theta(5) + theta(2) + theta(3) + theta(1) - 5 *theta(4)); % control law for the second robot 
    %% u(5) = K*(theta(6) + theta(2) + theta(4) + theta(3) + theta(1) - 5 *theta(5)); % control law for the second robot 
    %% u(6) = K*(theta(2) + theta(5) + theta(4) + theta(3) + theta(1) - 5 *theta(6)); % control law for the second robot 

  %partB 
  u(1) = K*(theta(2) - theta(1));
  u(2) = K*(theta(1) + theta(3) + theta(5) - 3*theta(2));
  u(3) = K*(theta(2) - theta(3));
  u(4) = K*(theta(5) + theta(6) - 2*theta(4));
  u(5) = K*(theta(6) + theta(4) + theta(2) - 3 * theta(5));
  u(6) = K*(theta(5) + theta(4) - 2*theta(6));
    
    %partC
    % u(1) = K*(theta(2) - theta(1));
    %u(2) = K*(theta(1) + theta(3) - 2*theta(2));
    %u(3) = K*(theta(2) - theta(3));
    %u(4) = K*(theta(5) + theta(6) - 2*theta(4));
    %u(5) = K*(theta(6) + theta(4) - 2*theta(5));
    %u(6) = K*(theta(5) + theta(4) - 2*theta(6));
 
  
 
%% update 
 
x = x + v.*cos(theta)*dt;
y = y + v.*sin(theta)*dt;
theta = theta + u*dt;  % Implmentation using Euler's method in matrix notation
         
%% store for offline plotting
xStore(iStep,:) = x;    
yStore(iStep,:) = y;    
thetaStore(iStep,:) = theta; 
uStore(iStep,:) = u; 
time(iStep,:) = t;
iStep = iStep + 1;
%% 
drawnow

end

% plotting

%% trajactories of robots

figure(1)
plot(xStore,yStore,'LineWidth',2); hold on
set(gca,'fontsize',14,'Fontname','Helvetica');
xlabel('X(m)');
ylabel('Y(m)');
grid on
axis equal
axis square

 %% velocity directions 
 
figure(2)
plot(time(1:end,:), thetaStore(1:end,:), 'LineWidth',2); hold on
set(gca,'fontsize',14,'Fontname','Helvetica');
xlabel('time (sec)');
ylabel('$\theta$');
grid on
axis equal
axis square

 %% control inputs 
 
figure(3)
plot(time(1:end,:), uStore(1:end,:), 'LineWidth',2); hold on
set(gca,'fontsize',14,'Fontname','Helvetica');
xlabel('time (sec)');
ylabel('u');
grid on
axis equal
axis square

toc