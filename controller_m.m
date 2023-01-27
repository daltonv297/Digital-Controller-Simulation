clc
close all

Kp = 2.4; % proportional gain
Ki = 0.05; % integral gain
Kd = 0.4; % derivative gain

tmax = 5; % s

T = 0.15; % sampling time, s

K_b = 1/4.6027; % DC motor voltage coefficient
tau_m = 0.22; % time constant

s = tf('s');

G = 1/K_b / (s*(tau_m*s+1)); % DC motor transfer function


u = stepFun(1, 1, T, tmax); % commanded theta
e = zeros(1, length(u)); % feedback error (theta_c - theta)
p = zeros(1, length(u)); % proportional term
I = zeros(1, length(u)); % integral term
d = zeros(1, length(u)); % derivative term
c = zeros(1, length(u)); % controller output
w = zeros(1, length(u)); % omega
theta = zeros(1, length(u)); 

for i = 2:length(u) % starting from t(1) because t(0) is known
    e(i) = u(i) - theta(i-1);
    p(i) = Kp*e(i);
    I(i) = Ki*T/2 * (e(i) + e(i-1)) + I(i-1);
    d(i) = Kd*w(i-1);
    c(i) = p(i) + I(i) - d(i);
    w(i) = (c(i) + c(i-1) - w(i-1)*(K_b - 2*K_b*tau_m/T))/(2*K_b*tau_m/T + K_b);
    theta(i) = T/2 * (w(i)+w(i-1)) + theta(i-1);
end

t = 0:T:tmax;

hold on
plot(t, theta)
plot(out.tout, out.theta)

xlabel('Time (s)')
ylabel('Motor angle (radians)')
legend('MATLAB', 'Simulink')

figure
error = abs(interp1(out.tout, out.theta, t) - theta);
plot(t, error)
xlabel('Time (s)')
ylabel('True error (radians)')


function u = stepFun(stepTime, stepVal, T, Tmax)
    t = 0:T:Tmax;
    u = stepVal*(t>=stepTime);
end