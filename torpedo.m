%% Globals

kn2mjs=1852/3600; % Umrechnung knoten m/s
Kp = 1; % Verstärkung
T_torpedo = 10; % Zeitkonstante Torpedo
t_max = 500; % max. Laufzeit Torpedo in Sekunden
dt = 0.001; % Abtastzeit
phi_zz = 30 * (pi/180);
sim_length = t_max/dt; % Simulationslenge
sim_vec = zeros(1,sim_length); % Simulations Basisvektor
time_vec = sim_vec; % Zeitvektor

v_torpedo = 50*kn2mjs; % Geschw. Torpedo in Knoten --> m/s
v_zerstoerer = 30*kn2mjs; % Geschw. Schiff in Knoten --> m/s

%% Vectors

x1 = sim_vec;
x2 = sim_vec;
x3 = sim_vec;
x4 = sim_vec;
x5 = sim_vec;
x6 = sim_vec;

x_torpedo = sim_vec; % X-Richtung Torpedo 
y_torpedo = sim_vec; % Y-Richtung Torpedo
y_torpedo(1) = 4000; % Startentfernung Torpedo -> Zerst�rer
vx_torpedo = sim_vec; % Geschwindigkeit X-Richtung Torpedo
vy_torpedo = sim_vec; % Geschwindigkeit Y-Richtung Torpedo

x_zerstoerer = sim_vec; % X-Richtung Zerst�rer
y_zerstoerer = sim_vec; % y-Richtung Zerst�rer
vx_zerstoerer = sim_vec; % Geschwindigkeit X-Richtung Zerst�rer
vy_zerstoerer = sim_vec; % Geschwindigkeit Y-Richtung Zerst�rer

phi_ist = sim_vec; % Winkel Torpedo ist
phi_soll = sim_vec; % Winkel Torpedo soll
phi_zerstoerer = sim_vec; % Winkel Zerstörer

k = 1;

%% Regler

% Rudermaschine
K_RM = 1;
T_RM = 0.2;
% Rudermaschine
K_IS = 1;
T_Nomoto = 4;
% Messgeber
K_Mess = 1;
T_Mess = 0.5;
% Regler
T_J = 200;
K_Reg = 1; 
T_D = T_Nomoto;
T_V = 0.1;

% Regler Nominator (N) und Denuminator (D)
G_PID_N = [T_J * (T_V + T_D) T_V + T_J 1 ];
G_PID_D = [T_V * T_J T_J 0];

G_RM_N = K_RM;
G_RM_D = [T_RM 1];

G_PHI_N = K_IS;
G_PHI_D = [T_Nomoto 1 0];

G_MESS_N = K_Mess;
G_MESS_D = [T_Mess 1];

% Übertragungsfunktion der einzelnen Glieder über tf()
G_PID = tf(G_PID_N, G_PID_D);
G_RM = tf(G_RM_N, G_RM_D);
G_PHI = tf(G_PHI_N, G_PHI_D);
G_MESS = tf(G_MESS_N, G_MESS_D);
G_phi_is = feedback(series(G_REGLER1, series(G_RM, G_PHI)),G_MESS);
%sisotool(G_RM * G_PHI,G_PID,G_MESS);

%% Zustandsraum

[G_phi_is__n, G_phi_is__d] = tfdata(G_phi_is);

[SS_A_CNT, SS_B_CNT, SS_C_CNT, SS_D_CNT] = tf2ss(cell2mat(G_phi_is__n), cell2mat(G_phi_is__d));

%% Calculation

while( (sqrt((x_zerstoerer(k) - x_torpedo(k))^2 + (y_torpedo(k) - y_zerstoerer(k))^2)>=5) && ((k * dt)<t_max))
    
    % Berechnung Winkel
    phi_soll(k) = atan2((x_zerstoerer(k) - x_torpedo(k)), (y_torpedo(k) - y_zerstoerer(k)));

    x1(k + 1) = x1(k) + dt * (SS_A_CNT(1,1) * x1(k)+(SS_A_CNT(1,2)) * x2(k) + (SS_A_CNT(1,3)) * x3(k) + (SS_A_CNT(1,4)) * x4(k)+(SS_A_CNT(1,5)) * x5(k) + (SS_A_CNT(1,6)) * x6(k) + SS_B_CNT(1) * phi_soll(k));
    x2(k + 1) = x2(k) + dt * (SS_A_CNT(2,1) * x1(k)+(SS_A_CNT(2,2)) * x2(k) + (SS_A_CNT(2,3)) * x3(k) + (SS_A_CNT(2,4)) * x4(k)+(SS_A_CNT(2,5)) * x5(k) + (SS_A_CNT(2,6)) * x6(k) + SS_B_CNT(2) * phi_soll(k));
    x3(k + 1) = x3(k) + dt * (SS_A_CNT(3,1) * x1(k)+(SS_A_CNT(3,2)) * x2(k) + (SS_A_CNT(3,3)) * x3(k) + (SS_A_CNT(3,4)) * x4(k)+(SS_A_CNT(3,5)) * x5(k) + (SS_A_CNT(3,6)) * x6(k) + SS_B_CNT(3) * phi_soll(k));
    x4(k + 1) = x4(k) + dt * (SS_A_CNT(4,1) * x1(k)+(SS_A_CNT(4,2)) * x2(k) + (SS_A_CNT(4,3)) * x3(k) + (SS_A_CNT(4,4)) * x4(k)+(SS_A_CNT(4,5)) * x5(k) + (SS_A_CNT(4,6)) * x6(k) + SS_B_CNT(4) * phi_soll(k));
    x5(k + 1) = x5(k) + dt * (SS_A_CNT(5,1) * x1(k)+(SS_A_CNT(5,2)) * x2(k) + (SS_A_CNT(5,3)) * x3(k) + (SS_A_CNT(5,4)) * x4(k)+(SS_A_CNT(5,5)) * x5(k) + (SS_A_CNT(5,6)) * x6(k) + SS_B_CNT(5) * phi_soll(k));
    x6(k + 1) = x6(k) + dt * (SS_A_CNT(6,1) * x1(k)+(SS_A_CNT(6,2)) * x2(k) + (SS_A_CNT(6,3)) * x3(k) + (SS_A_CNT(6,4)) * x4(k)+(SS_A_CNT(6,5)) * x5(k) + (SS_A_CNT(6,6)) * x6(k) + SS_B_CNT(6) * phi_soll(k));
    phi_ist(k) = SS_C_CNT(1) * x1(k) + SS_C_CNT(2) * x2(k) + SS_C_CNT(3) * x3(k) + SS_C_CNT(4) * x4(k) + SS_C_CNT(5) * x5(k) + SS_C_CNT(6) * x6(k);

    phi_ist(k + 1) = phi_ist(k) + dt * ( -(1/T_torpedo) * phi_ist(k) + (Kp/T_torpedo) * phi_soll(k));
    
    % Berechnung Geschwindigkeit
    vx_torpedo(k) = v_torpedo * sin(phi_ist(k)) * dt;
    vy_torpedo(k) = -v_torpedo * cos(phi_ist(k)) * dt;
    vx_zerstoerer(k) = v_zerstoerer * dt * cos(phi_zerstoerer(k));
    vy_zerstoerer(k) = -v_zerstoerer * dt * sin(phi_zerstoerer(k));
    
    % Berechnung Position
    x_torpedo(k + 1) = x_torpedo(k) + vx_torpedo(k);
    y_torpedo(k + 1) = y_torpedo(k) + vy_torpedo(k);
    x_zerstoerer(k + 1) = x_zerstoerer(k) + vx_zerstoerer(k);
    y_zerstoerer(k + 1) = y_zerstoerer(k) + vy_zerstoerer(k);
     
 
    if(sqrt((y_zerstoerer(k) - y_torpedo(k))^2 + (x_zerstoerer(k) - x_torpedo(k))^2)<=2000)
      x_zerstoerer(k + 1) = x_zerstoerer(k) + dt * cos(phi_zerstoerer(k) + phi_zz) * v_zerstoerer;
      y_zerstoerer(k + 1) = -y_zerstoerer(k) + dt * sin(phi_zerstoerer(k) + phi_zz) * v_zerstoerer;

      if mod(time_vec, 30)==0
        phi_zz = -phi_zz;
      end
    end
    
    time_vec(k) = (k - 1) * dt;
    k = k + 1;
end

plot (x_torpedo(1:k),y_torpedo(1:k),'r')
hold on
plot (x_zerstoerer(1:k),y_zerstoerer(1:k),'b')
grid on
