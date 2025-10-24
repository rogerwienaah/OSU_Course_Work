% Cart-Pole LQR Control - Question 1 of Pset3

% System Parameters
M = 5;     
m = 1;      
L = 2;     
b = 1;      
g = 9.8;   


% Linearized State-Space Matrices - Adapted from slides

A = [0   1   0       0;
     0   -b/M m*g/M   0;
     0   0   0       1;
     0   -b/(M*L) g*(M+m)/(M*L)   0];

B = [0;
     1/M;
     0;
     1/(M*L)];



% LQR Controller Design
Q = diag([10, 1, 100, 10]);  % Weights on [x, x_dot, theta-pi, theta_dot]
R = 1;                      % Weight on control force

% LQR gain
[K, P, E] = lqr(A, B, Q, R);


% Initial Conditions
x0 = -1;
x_dot0 = 0;
theta0 = pi + 0.1;
theta_dot0 = 0;
state0 = [x0; x_dot0; theta0; theta_dot0];


tspan = linspace(0, 5, 100); % 10 seconds, 500 points

% % System Dynamics with LQR Control - linearized system
% odefun_linear = @(t, x) (A - B*K) * x;

% % Simulate the linearized system
% [t_linear, x_linear] = ode45(odefun_linear, tspan, state0);

% Nonlinear System Dynamics - to comparison
odefun_nonlinear = @(t, x) cart_pole_dynamics(x, M, m, L, b, g, K);

% Simulate the nonlinear system
[t_nonlinear, x_nonlinear] = ode45(odefun_nonlinear, tspan, state0);


% ------------------------sane point ------


% --- Generate 100 Random Stable Eigenvalues and Simulate ---
num_random = 100;
all_x = zeros(length(tspan), num_random + 1); % Store x for all sims
all_x_dot = zeros(length(tspan), num_random + 1);
all_theta = zeros(length(tspan), num_random + 1);
all_theta_dot = zeros(length(tspan), num_random + 1);

% all_x(:, 1) = x_linear(:, 1);  % Store LQR linear response
% all_x_dot(:, 1) = x_linear(:, 2);
% all_theta(:, 1) = x_linear(:, 3) + pi;
% all_theta_dot(:, 1) = x_linear(:, 4);

all_x(:, 2) = x_nonlinear(:, 1); % Store LQR nonlinear response
all_x_dot(:, 2) = x_nonlinear(:, 2);
all_theta(:, 2) = x_nonlinear(:, 3) + pi;
all_theta_dot(:, 2) = x_nonlinear(:, 4);


for i = 1:num_random
    % Generate random stable eigenvalues
    eigenvalues = -3.5 + (3.5 - 0.5) * rand(4, 1);  % Eigenvalues in [-3.5, -0.5]
    eigenvalues = real(eigenvalues); % Ensure real eigenvalues
    
    % Create K matrix from eigenvalues (pole placement)
    K_random = place(A, B, eigenvalues);
    
    % Simulate nonlinear system with random K
    odefun_random = @(t, x) cart_pole_dynamics(x, M, m, L, b, g, K_random);
    [~, x_random] = ode45(odefun_random, tspan, state0);
    
    all_x(:, i+1) = x_random(:, 1);
    all_x_dot(:, i+1) = x_random(:, 2);
    all_theta(:, i+1) = x_random(:, 3) + pi;
    all_theta_dot(:, i+1) = x_random(:, 4);
end

% --- Plotting ---
figure;

% Plot x
subplot(2, 2, 1);
plot(tspan, all_x, 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Cart Position x (m)');
title('Cart Position');
grid on;
hold on;
plot(tspan, all_x(:, 2), 'r', 'LineWidth', 2); % LQR nonlinear
hold off;

% Plot x_dot
subplot(2, 2, 2);
plot(tspan, all_x_dot, 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Cart Velocity x_dot (m/s)');
title('Cart Velocity');
grid on;
hold on;
plot(tspan, all_x_dot(:, 2), 'r', 'LineWidth', 2); % LQR nonlinear
hold off;

% Plot theta
subplot(2, 2, 3);
plot(tspan, all_theta, 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Pendulum Angle theta (rad)');
title('Pendulum Angle');
grid on;
hold on;
plot(tspan, all_theta(:, 2), 'r', 'LineWidth', 2); % LQR nonlinear
hold off;

% Plot theta_dot
subplot(2, 2, 4);
plot(tspan, all_theta_dot, 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Pendulum Angular Velocity theta_dot (rad/s)');
title('Pendulum Angular Velocity');
grid on;
hold on;
plot(tspan, all_theta_dot(:, 1), 'k', 'LineWidth', 2); % LQR linear
plot(tspan, all_theta_dot(:, 2), 'r', 'LineWidth', 2); % LQR nonlinear
hold off;


% Adapted from slides

function dx_dt = cart_pole_dynamics(x, M, m, L, b, g, K)
    % Full nonlinear dynamics of the cart-pole system
    Sx = sin(x(3));
    Cx = cos(x(3));
    
    u = -K * x;  % LQR control force

    D = m*L*L*(M+m*(1-Cx^2));
    dx_dt(1,1) = x(2);
    dx_dt(2,1) = (1/D)*(-m^2*L^2*g*Cx*Sx + m*L^2*(m*L*x(4)^2*Sx - b*x(2))) + m*L*L*(1/D)*u;
    dx_dt(3,1) = x(4);
    dx_dt(4,1) = (1/D)*((m+M)*m*g*L*Sx - m*L*Cx*(m*L*x(4)^2*Sx - b*x(2))) - m*L*Cx*(1/D)*u;

end



% --- Function Definitions ---

% function dx_dt = cart_pole_dynamics(x, M, m, L, b, g, K)
%     % Nonlinear cart-pole dynamics with LQR control
% 
%     cart_pos = x(1);
%     cart_vel = x(2);
%     pend_angle = x(3);
%     pend_vel = x(4);
% 
%     u = -K * x;  % LQR control force
% 
%     % Nonlinear equations of motion
%     denom = L*(M + m - m*cos(pend_angle)^2);
% 
%     x_ddot = (u + m*L*sin(pend_angle)*pend_vel^2 - m*g*cos(pend_angle)*sin(pend_angle) - b*cart_vel) / ...
%              (M + m - m*cos(pend_angle)^2);
% 
%     theta_ddot = (-u*cos(pend_angle) - m*L*cos(pend_angle)*sin(pend_angle)*pend_vel^2 + ...
%                   (M + m)*g*sin(pend_angle) + b*cart_vel*cos(pend_angle)) / denom;
% 
%     dx_dt = [cart_vel;
%              x_ddot;
%              pend_vel;
%              theta_ddot];
% end