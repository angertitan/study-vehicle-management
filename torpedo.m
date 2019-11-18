%% Globals

kn2mjs=1852/3600; % Umrechnung knoten m/s
Kp = 1; % Verstärkung
T_torpedo = 30; % Zeitkonstante Torpedo
t_max = 500; % max. Laufzeit Torpedo in Sekunden
dt = 0.001; % Abtastzeit
sim_length = t_max/dt; % Simulationslenge
sim_vec = zeros(1,sim_length); % Simulations Basisvektor
time_vec = sim_vec; % Zeitvektor

v_torpedo = 50*kn2mjs; % Geschw. Torpedo in Knoten --> m/s
v_zerstoerer = 30*kn2mjs; % Geschw. Schiff in Knoten --> m/s

x_torpedo = sim_vec; % X-Richtung Torpedo 
y_torpedo = sim_vec; % Y-Richtung Torpedo
y_torpedo(1) = 4000; % Startentfernung Torpedo -> Zerst�rer
vx_torpedo = sim_vec; % Geschwindigkeit X-Richtung Torpedo
vy_torpedo = sim_vec; % Geschwindigkeit Y-Richtung Torpedo

x_zerstoerer = sim_vec; % X-Richtung Zerst�rer
y_zerstoerer = sim_vec; % y-Richtung Zerst�rer
vx_zerstoerer = sim_vec; % Geschwindigkeit X-Richtung Zerst�rer
vy_zerstoerer = sim_vec; % Geschwindigkeit Y-Richtung Zerst�rer

phi_ist = sim_vec;
phi_soll = sim_vec;
phi_zerstoerer = sim_vec;

k = 1;

T_NOMOTO = 4;

%% Regler

K_p = 1.0; % Verstärkung
T_D = T_NOMOTO; % sec
T_DV = 0.01; % sec
T_N = 200; % sec


G_REGLER1 = tf(K_p, 1) + tf(K_p*[T_D 0], [T_DV 1]) + tf(K_p,[T_N 0]);

G_REGLER2 = tf(K_p * [T_N*(T_D + T_DV) (T_N + T_DV) 1], [T_N * T_DV T_N 0]);


disp(G_REGLER1)
disp(G_REGLER2)
%% Calculation

while( (sqrt((x_zerstoerer(k) - x_torpedo(k))^2 + (y_torpedo(k) - y_zerstoerer(k))^2)>=5) && ((k*dt)<t_max))
    
    % Berechnung Winkel
    phi_soll(k) = atan2((x_zerstoerer(k) - x_torpedo(k)), (y_torpedo(k) - y_zerstoerer(k)));
    phi_ist(k+1) = phi_ist(k) + dt*(-(1/T_torpedo) * phi_ist(k) + (Kp/T_torpedo) * phi_soll(k));
    
    % Berechnung Geschwindigkeit
    vx_torpedo(k) = v_torpedo * sin(phi_ist(k)) * dt;
    vy_torpedo(k) = -v_torpedo * cos(phi_ist(k)) * dt;
    vx_zerstoerer(k) = v_zerstoerer * cos(phi_zerstoerer(k)) * dt;
    vy_zerstoerer(k) = -v_zerstoerer * sin(phi_zerstoerer(k)) * dt;
    
    % Berechnung Position
    x_torpedo(k+1) = x_torpedo(k) + vx_torpedo(k);
    y_torpedo(k+1) = y_torpedo(k) + vy_torpedo(k);
    x_zerstoerer(k+1) = x_zerstoerer(k)+vx_zerstoerer(k);
    y_zerstoerer(k+1) = y_zerstoerer(k)+vy_zerstoerer(k);
     
 
    time_vec(k)=(k-1)*dt;
    k=k+1;
    
end

plot (x_torpedo(1:k),y_torpedo(1:k),'r')
hold on
plot (x_zerstoerer(1:k),y_zerstoerer(1:k),'b')
grid on
