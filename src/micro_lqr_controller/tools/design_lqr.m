%% Standard discrete LQR design for the fixed-leg two-wheel robot
% State order: x = [pitch; pitch_rate; position; velocity]
% Model input: TOTAL axle torque, u_total = tau_left_forward + tau_right_forward
% The ROS 2 node builds this same model and solves the DARE at startup.

clear; clc;

%% Physical parameters -- replace with measured/CAD values
m = 2.54;                 % body mass participating in pitch [kg]
M = 0.26;                 % non-pitch translating mass [kg]
Jw = 0.0;                 % each wheel rotational inertia [kg*m^2]
h = 0.120;                % axle-to-body-COM distance [m]
I = 0.036576;             % body pitch inertia about COM [kg*m^2]
r = 0.030;                % wheel radius [m]
g = 9.80665;              % gravity [m/s^2]
Ts = 0.003;                % controller period [s]
use_course_legacy_b4 = false;

% Equivalent translational mass includes two wheel rotational inertias.
Meq = M + 2*Jw/r^2;
D = (Meq+m)*(m*h^2 + I) - m^2*h^2;

A = [0, 1, 0, 0;
     ((Meq+m)*m*g*h)/D, 0, 0, 0;
     0, 0, 0, 1;
     -(m^2*g*h^2)/D, 0, 0, 0];

if use_course_legacy_b4
    % Exact fourth B entry found in the uploaded course wheel_control.m.
    b4 = 1/((Meq+m)*r) - (m^2*h^2)/((Meq+m)*D*r);
else
    % Standard cart-pole result after solving the coupled equations.
    b4 = (m*h^2 + I)/(D*r);
end
B = [0; -(m*h)/(D*r); 0; b4];
C = eye(4);
Dmat = zeros(4,1);

%% Cost settings
Q = diag([1, 1, 1, 1]);
R = 10;                   % course example uses 1; 10 is safer for first hardware test

%% Exact zero-order-hold discretization and discrete LQR
sys_c = ss(A, B, C, Dmat);
sys_d = c2d(sys_c, Ts, 'zoh');
Ad = sys_d.A;
Bd = sys_d.B;
[K, P, poles] = dlqr(Ad, Bd, Q, R);

fprintf('State order: [pitch pitch_rate x x_dot]\n');
fprintf('Input: total axle torque [N*m]\n');
fprintf('K = [%.12g, %.12g, %.12g, %.12g]\n', K(1), K(2), K(3), K(4));
fprintf('max(abs(closed-loop poles)) = %.12g\n', max(abs(poles)));
fprintf('controllability rank = %d\n', rank(ctrb(Ad, Bd)));

fprintf('\nYAML manual-gain comparison block:\n');
fprintf('lqr.use_manual_gain: true\n');
fprintf('lqr.manual_k_pitch: %.12g\n', K(1));
fprintf('lqr.manual_k_pitch_rate: %.12g\n', K(2));
fprintf('lqr.manual_k_position: %.12g\n', K(3));
fprintf('lqr.manual_k_velocity: %.12g\n', K(4));

%% Optional linear closed-loop initial-condition simulation
x0 = [3*pi/180; 0; 0; 0];
N = round(5/Ts);
x = zeros(4, N+1);
u = zeros(1, N);
x(:,1) = x0;
for k = 1:N
    u(k) = -K*x(:,k);
    x(:,k+1) = Ad*x(:,k) + Bd*u(k);
end
t = (0:N)*Ts;
figure('Name','Discrete LQR linear model');
subplot(3,1,1); plot(t, x(1,:)*180/pi); grid on; ylabel('pitch [deg]');
subplot(3,1,2); plot(t, x(3,:)); grid on; ylabel('x [m]');
subplot(3,1,3); stairs(t(1:end-1), u); grid on; ylabel('u total [N m]'); xlabel('time [s]');
