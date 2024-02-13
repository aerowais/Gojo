clc
clear all
close all
dt = 1/50
x0 =    [35;
        0;
        0;
        0;
        0;
        0;
        0;
        0;
        0];

u =     [0;
        0;
        0;
        0];

u1min = -12*pi/180;             % Aileron
u1max = 12*pi/180;

u2min = -25*pi/180;             % Elevator
u2max = 25*pi/180;

u3min = -5*pi/180;             % Rudder
u3max = 5*pi/180;

u4min = 0;                      % Throttle
u4max = 1;


TF = 60;

% Run the simulation
sim('UAV_Simulation.slx')

% Plotting the output
t = ans.simX.Time;

u1 = ans.simU.Data(:,1);
u2 = ans.simU.Data(:,2);
u3 = ans.simU.Data(:,3);
u4 = ans.simU.Data(:,4);

x1 = ans.simX.Data(:,1);
x2 = ans.simX.Data(:,2);
x3 = ans.simX.Data(:,3);
x4 = ans.simX.Data(:,4);
x5 = ans.simX.Data(:,5);
x6 = ans.simX.Data(:,6);
x7 = ans.simX.Data(:,7);
x8 = ans.simX.Data(:,8);
x9 = ans.simX.Data(:,9);

% % Ploting the variation of the Control output
% figure
% subplot(4,1,1)
% plot(t,u1)
% legend('Aileron deflection')
% grid on
% 
% subplot(4,1,2)
% plot(t,u2)
% legend('Elevator deflection ')
% grid on
% 
% subplot(4,1,3)
% plot(t,u3)
% legend('Rudder deflection')
% grid on
% 
% subplot(4,1,4)
% plot(t,u4)
% legend('Throttle deflection')
% grid on

% Ploting the variation of states

figure

% u,v,w
subplot(3,3,1)
plot(t,x1)
legend('u_b [m/s]')
xlabel('time [s]')
ylabel('u_b [m/s]')
grid on

subplot(3,3,4)
plot(t,x2)
legend('v_b [m/s]')
xlabel('time [s]')
ylabel('v_b [m/s]')
grid on

subplot(3,3,7)
plot(t,x3)
legend('w_b [m/s]')
xlabel('time [s]')
ylabel('w_b [m/s]')
grid on

% p,q,r

subplot(3,3,2)
plot(t,x4)
legend('p [rad/s]')
xlabel('time [s]')
ylabel('p [rad/s]')
grid on

subplot(3,3,5)
plot(t,x5)
legend('q [rad/s]')
xlabel('time [s]')
ylabel('q [rad/s]')
grid on

subplot(3,3,8)
plot(t,x6)
legend('r [rad/s]')
xlabel('time [s]')
ylabel('r [rad/s]')
grid on

% phi,theta,psi
subplot(3,3,3)
plot(t,x7)
legend('phi [rad]')
xlabel('time [s]')
ylabel('phi [rad]')
grid on

subplot(3,3,6)
plot(t,x8)
legend('theta [rad]')
xlabel('time [s]')
ylabel('theta [rad]')
grid on

subplot(3,3,9)
plot(t,x9)
legend('psi [rad]')
xlabel('time [s]')
ylabel('psi [rad]')
grid on












