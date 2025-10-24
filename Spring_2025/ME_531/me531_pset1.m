%% question 1b

% the state-space matrices

% A = [0,   1,    0,  0,  0,  0; 
%     -2, -2,  1,   1,   0,  0; 
%     0,   0,   0,   1,   0, 0; 
%     1,    1,   -2, -1, 1,   0; 
%     0,   0,   0,  0,  0,  1; 
%     0,    0,   1,   0, -2, -1];
% 
% B = [0; 1; 0; 0; 0; 0];
% C = eye(6);  % output matrix
% D = zeros(6, 1); % feedthrough matrix
% 
% % state-space model
% sys = ss(A, B, C, D);
% 
% % impulse response for 20 seconds
% t = 0:0.01:20;
% [y, t] = impulse(sys, t);
% 
% % Plot position states
% figure;
% plot(t, y(:, 1), 'r', t, y(:, 3), 'g', t, y(:, 5), 'b');
% xlabel('Time (s)');
% ylabel('Position (m)');
% legend('Manipulator', 'Sensor', 'Environment');
% title('Impulse Response - Position States');
% grid on;



%% question 3

V1 = 100; 
V2 = 100; 
F = 2;    % gal/min
y1_0 = 0;   
y2_0 = 150; 

% time
t = linspace(0, 200, 1000); % Time from 0 to 200 minutes

% fertilizer amounts
y1 = 75 * (1 - exp(-0.04 * t));
y2 = 75 * (1 + exp(-0.04 * t));

% Plot
plot(t, y1, 'b', t, y2, 'r');
xlabel('Time (minutes)');
ylabel('Fertilizer Amount (lbs)');
legend('Tank T1', 'Tank T2');
title('Fertilizer Content in Tanks');
grid on;