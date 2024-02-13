function XDOT = UAV_model_aerosonde(X,U)


% ============= State and Control vectors ============= %

u       = X(1,1);
v       = X(2,1);
w       = X(3,1);

p       = X(4,1);
q       = X(5,1);
r       = X(6,1);

phi     = X(7,1);
theta   = X(8,1);
psi     = X(9,1);

% pn      = X(10);    % north velocity in ned frame
% pe      = X(11);    % east velocity in ned frame
% pd      = X(12);    % down velocity in ned frame

u1 = U(1,1); %d_A aileron
u2 = U(2,1); %d_T elevator
u3 = U(3,1); %d_R rudder
u4 = U(4,1); %d_th throttle


%=======Control limits and Saturation=======%
% %can be changed in simulink
% %{
% u1min = -25*pi/180; %Ailerons limits
% u1max = 25*pi/180;
% 
% u2min = -25*pi/180; %Elevator limits
% u2max = 10*pi/180;
% 
% u3min = -30*pi/180; % Rudder limits
% u3max = 30*pi/180;
% 
% u4min = 0.5*pi/180; % Throttle 1 limits
% u4max = 10*pi/180;
% 
% u5min = 0.5*pi/180; % Throttle 2 limits
% u5max = 10*pi/180;
% %}
% % %Saturation 
% % %{
% if(u1>u1max)
%     u1 = u1max;
% elseif(u1<u1min)
%     u1 = u1min;
% end
% 
% if(u2>u2max)
%     u2 = u2max;
% elseif(u2<u2min)
%     u2 = u2min;
% end
% 
% if(u3>u3max)
%     u3 = u3max;
% elseif(u3<u3min)
%     u3 = u3min;
% end
% 
% if(u4>u4max)
%     u4 = u4max;
% elseif(u4<u4min)
%     u4 = u4min;
% end


% %}


% ============= Constants ============= %

%nominal UAV constants
m = 11; %aircraft mass (kg)

%Aero
c = 0.19; % Mean aerodynamic chord (m)
b = 2.9;  % span of wing (m)
S = 0.55;    % Wing planform area (m^2)
e = 0.9;     % Oswald  efficiency factor


% Inertia matrix
Jx = 0.8244;    % x moment of inertia (kg*m^2)
Jy = 1.135;     % y moment of inertia (kg*m^2)
Jz = 1.759;     % z moment of inertia (kg*m^2)

Jxz = 0.1204;    % xz product of inertia (kg*m^2)

S_prop  = 0.2027;
k_motor = 80;
k_T_p    = 0;
k_omega = 0;


%Miscellaneous constants
rho = 1.2682;        %air density (kg/m^3)
g = 9.81;           %gravitational acceleration (m/s^2)
alpha_0 = -0.4712;    %zero lift AoA (rad)
M_constant = 50;
epsilon = 0.1592;


% Longitudinal Coefficients

C_L_0 = 0.23;
C_D_0 = 0.043;
C_m_0 = 0.0135;

C_L_alpha = 5.61;
C_D_alpha = 0.03;
C_m_alpha = -2.74;

C_L_q = 7.95;
C_D_q = 0;
C_m_q = -38.21;

C_L_delta_e = 0.13;
C_D_delta_e = 0.0135;
C_m_delta_e = -0.99;

C_prop = 1.0;

C_D_p = 0.0;




% Lateral coefficients

C_Y_0 = 0;
C_l_0 = 0;
C_n_0 = 0;

C_Y_beta = -0.83;
C_l_beta = -0.13;
C_n_beta = 0.073;

C_Y_p = 0;
C_l_p = -0.51;
C_n_p = -0.069;

C_Y_r = 0;
C_l_r = 0.045;
C_n_r = -0.095;

C_Y_delta_a = 0.075;
C_l_delta_a = 0.17;
C_n_delta_a = -0.011;

C_Y_delta_r = 0.19;
C_l_delta_r = 0.0024;
C_n_delta_r = -0.069; 




%===========Intermediate Variables==========%

% compute air data
Va = sqrt((u^2) + (v^2) + (w^2));

alpha = atan2(w,u); %Angle of attack
beta = asin(v/Va);  %Side slip

Q = 0.5*rho*(Va^2);

% Calculation of coefficient of lift and drag, assumed to linear, as the
% flight paths are assumed to be set in a 2D environment

C_L = C_L_0 + C_L_alpha * alpha;
C_D = C_D_0 + C_D_alpha * alpha;

% Intemediate coefficients

C_X = -C_D*cos(alpha) + C_L*sin(alpha);
C_X_q = -C_D_q*cos(alpha) + C_L_q*sin(alpha);

C_X_delta_e  = -C_D_delta_e*cos(alpha) + C_L_delta_e*sin(alpha);

C_Z = -C_D*sin(alpha) - C_L*cos(alpha);

C_Z_q =  -C_D_q*sin(alpha) - C_L_q*cos(alpha);

C_Z_delta_e = -C_D_delta_e*sin(alpha) - C_L_delta_e*cos(alpha);


% compute external forces and moments on aircraft
Force(1,1) = -m*g*sin(theta) + Q*S*(C_X + C_X_q*(c/(2*Va))*q + C_X_delta_e*u2) + 0.5*rho*S_prop*C_prop*( ((k_motor*u4)^2) - (Va^2) ) ;            % force in body x-direction

Force(2,1) = m*g*cos(theta)*sin(phi) + Q*S*(C_Y_0 + C_Y_beta*beta + C_Y_p*(b/(2*Va))*p + C_Y_r*(b/(2*Va))*r + C_Y_delta_a*u1 + C_Y_delta_r*u3); ; % force in body y-direction

Force(3,1) =  m*g*cos(theta)*cos(phi) + Q*S*(C_Z + C_Z_q*(c/(2*Va))*q + C_Z_delta_e*u2);                                                                % force in body z-direction    
    

Moment(1,1) = Q*S*b*(C_l_0 + C_l_beta*beta + C_l_p*(b/(2*Va))*p + C_l_r*(b/(2*Va))*r + C_l_delta_a*u1 + C_l_delta_r*u3) - k_T_p*((k_omega*u4)^2); % roll moment: l

Moment(2,1) = Q*S*c*(C_m_0 + C_m_alpha*alpha + C_m_q*(c/(2*Va))*q + C_m_delta_e*u2); % pitch moment: m

Moment(3,1) = Q*S*b*(C_n_0 + C_n_beta*beta + C_n_p*(b/(2*Va))*p + C_n_r*(b/(2*Va))*r + C_n_delta_a*u1 + C_n_delta_r*u3); % yaw moment: n

%=========== The six-degree-of-freedom, 12-state model for the UAV kinematics ===========%

% Intemediate constants

gamma = Jx*Jz - (Jxz^2);

gamma_1 = (Jxz*(Jx - Jy + Jz))/gamma;

gamma_2 = (Jz*(Jz - Jy) + (Jxz^2))/gamma;
 
gamma_3 = Jz/gamma;

gamma_4 = Jxz/gamma;

gamma_5 = (Jz - Jx)/Jy;

gamma_6 = Jxz/Jy;

gamma_7 = ( (Jx-Jy)*Jx + (Jxz^2) )/gamma;

gamma_8 = Jx/gamma;

% A_bi = [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
%          cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
%          -sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)];

f_x = Force(1,1);
f_y = Force(2,1);
f_z = Force(3,1);

l = Moment(1,1);
mo = Moment(2,1);
n = Moment(3,1);



%-----------Calculating the output of the function, the X_dot vector--------


%-----------------------
% pi_dot = A_bi * [u;v;w];
%-----------------------

% pn_dot = pi_dot(1,1);
% pe_dot = pi_dot(2,1);
% pd_dot = pi_dot(3,1);


u_dot = r*v - q*w + (1/m)* f_x;
v_dot = p*w - r*u + (1/m)* f_y;
w_dot = q*u - p*v + (1/m)* f_z;

%------------------------------------------------------------------
angle_dot = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);
             0 cos(phi) -sin(phi);
             0 sin(phi)/cos(theta) cos(phi)/cos(theta)] * [p;q;r];
%------------------------------------------------------------------
p_dot = gamma_1*p*q - gamma_2*q*r + gamma_3*l + gamma_4*n;
q_dot = gamma_5*p*r - gamma_6*( (p^2) - (r^2) ) + (1/Jy)*mo;
r_dot = gamma_7*p*q - gamma_1*q*r + gamma_4*l + gamma_8*n;

phi_dot   = angle_dot(1,1);
theta_dot = angle_dot(2,1);
psi_dot   = angle_dot(3,1);


%------------ Arranging all the terms to give the XDOT vector----------%

XDOT = [u_dot;
        v_dot;
        w_dot;
        p_dot;
        q_dot;
        r_dot;
        phi_dot;
        theta_dot;
        psi_dot];
        
end
