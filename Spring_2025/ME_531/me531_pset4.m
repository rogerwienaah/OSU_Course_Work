% ME 531 Pset 4 - Question 1
%Full state observer design

clear all;
A = [1, 4; -5, 10];
B = [0; 1];
C = [1 -4];

%check whether this system is observable
obsv_mat = obsv(A, C);
obsv_mat_rank = rank(obsv_mat);

if obsv_mat_rank == length(A)
    disp("System is fully observable")
else
    disp("System is not fully observable")
end

% Desired poles
op1 = -1;
op2 = -1.0000001; %MATLAB error when op2 is set to -1, so i made it a bit smaller

% the observer gain matrix .
L = place(A',C',[op1 op2]);

% initial estimation error
e0 = [1; 1];

% Error dynamics
sys = A-(L'*C);
f = @(t,e) [sys(1,1)*e(1)+sys(1,2)*e(2); sys(2,1)*e(1)+sys(2,2)*e(2)];

%Error calc.
[ts,ys] = ode45(f,[0,20],e0); %simulate response for 20 sec
plot(ts,ys(:,1),ts,ys(:,2))
xlabel('Time (sec)')
ylabel('Estimation Error')
title('Estimation Error Response')
legend('e_1','e_2')
grid on



% 
% %% Question 2:
% 
% m = 5;
% k = 2;
% b = 2;
% 
% A = [0 1; -k/m -b/m];
% B = [0; 1/m];
% C = [1 0];
% D = 0;
% % sys = ss(A,B,C,D);
% 
% % Desired performance
% OS_percent = 4; % perc. overshoot = 4%
% Ts = 2;         % 2s Settling time
% 
% % Desired damping ratio (zeta) and natural frequency (omega_n)
% OS = OS_percent / 100;
% zeta = -log(OS) / sqrt(pi^2 + log(OS)^2);
% omega_n = 4 / (zeta * Ts);
% 
% % Desired closed-loop poles for the controller
% p_control = roots([1 2*zeta*omega_n omega_n^2]);
% 
% % state feedback gain matrix (K) using pole placement
% K = place(A, B, p_control);
% 
% % Q = diag([100, 1]);
% % R = 1;
% % 
% % [K, S, e] = lqr(sys,Q,R);
% 
% %initial conditions
% x = 1;
% x_dot = 1;
% 
% x0 = [x, x_dot];
% 
% tspan = linspace(0, 10, 400);
% 
% odefun = @(t, x) spring_mass_damper(x, A, B, K);
% 
% % System simulation
% [t, x] = ode45(odefun, tspan, x0);
% 
% % % Control input
% % u = -K * x';
% 
% 
% % Spring_mass_damper dynamics with LQR control
% function dx_dt = spring_mass_damper(x, A, B, K)
%     ref = [0; 0];
%     u = -K * (x-ref);
%     dx_dt = A * x + B * u;
% end
% 
% 
% % Plot x
% subplot(2, 2, 1);
% plot(t, x(:, 1), 'b');
% xlabel('Time (s)');
% ylabel('pos');
% grid on;
% 
% % Plot x_dot
% subplot(2, 2, 2);
% plot(t, x(:, 2), 'r');
% xlabel('Time (s)');
% ylabel('vel');
% grid on;
% 
% 
% 
% %% observer part
% 
% % For a settling time of approximately 0.5 sec
% zeta = 0.7156; % damping ratio
% omega = 2.7947; % natural frequency
% % Desired poles
% op1 = -zeta*omega+(omega*(1-zeta^2)^0.5)*1i;
% op2 = -zeta*omega-(omega*(1-zeta^2)^0.5)*1i;
% % Find the observer gain. Note, must use the transpose of A and C.
% L = place(A',C',[op1 op2]);
% % See response to initial estimation error
% e0 = [0.5; -0.5];
% % Error dynamics
% sys = A-(L'*C);
% f = @(t,e) [sys(1,1)*e(1)+sys(1,2)*e(2); sys(2,1)*e(1)+sys(2,2)*e(2)];
% %
% [ts,ys] = ode45(f,[0,10],e0);
% plot(ts,ys(:,1),ts,ys(:,2))
% xlabel('Time (sec)')
% ylabel('Estimation Error')
% legend('e_1','e_2')
