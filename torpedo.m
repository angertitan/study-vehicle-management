%% --- Reset --- %%
clear all
close all

%% --- Globale Parameter --- %%
% -- Umrechnungen -- %
kn2mjs = 1852/3600; % [-] Umrechnungsfaktor Knoten -> m/s

% -- Simulations Parameter -- %
dt = 0.001; % [s] Abstastrate
t_max = 500; % [s] maximale Laufzeit Torpedo
sim_length = t_max/dt; % [-] maximale Simulationslänge
k = 1; % [-] Simulationsindex
timestamp = 0; % [s] 

% -- Torpedo Parameter -- %
K_p = 1; % [-] Verstärkung
T_t = 15; % [s] Zeitkonstante Torpedo
v_t = 50 * kn2mjs; % [m/s] Geschwindigkeit Torpedo (Knoten -> m/s)
y_t0 = 4000; % [m] Startentfernung Torpedo <-> Zerstörer

% -- Zerstörer Parameter -- %
v_z = 30 * kn2mjs; % [m/s] Geschwindigkeit Zerstörer (Knoten -> m/s)
phi_z_zz = deg2rad(20); % [rad] Zick Zack Winkel der Zerstörers
phi_flucht = deg2rad(45); % [rad] Fluchtwinkel Zerstörer

%% --- Vektoren --- %%
% -- Simulations Vektoren -- %
base_vec = zeros(1, sim_length); % Basisvektor
time_vec = zeros(1, sim_length); % [s] Zeitvektor

% -- Torpedo Vektoren -- %
x_t = base_vec; % [m] X-Richtungsvektor Torpedo
y_t = base_vec; % [m] Y-Richtungsvektor Torpedo
y_t(1) = y_t0; % [m] Startentfernung setzen

phi_soll = base_vec; % [rad] Sollwinkel Torpedo
% phi_t_ist = base_vec; % [rad] Istwinkel Ruder Torpedo (in Rad)
% phi_t_ist__grad = base_vec; % [grad] Istwinkel Ruder Torpedo (in Grad)

% -- Zerstörer Vektoren -- %
x_z = base_vec; % [m] X-Richtungsvektor Zerstörer
y_z = base_vec; % [m] Y-Richtungsvektor Zerstörer

phi_z = base_vec; % [rad] Winkel Zerstörer

% --Zustandsvektoren -- %

% Vektoren
u_w = base_vec;
u_REG = base_vec;
u_INT = base_vec;
u_GES = base_vec;
e_phi = base_vec;
x1 = base_vec;
x2 = base_vec;
x3 = base_vec;
phi_ist = base_vec;
x1_MESS = base_vec;
phi_MESS = base_vec;

%% --- Modellbildung --- %%
% -- Modell Parameter -- %
% Rudermaschine
K_RM = 1; % [-] Verstärkung Rudermaschine 
T_RM = 0.2; % [s] Zeitkonstante Rudermaschine

% Kursverhalten Torpedo
K_IS = 1; % [-] Verstärkung Torpedo 
T_Nomoto = 4; % [s] Zeitkonstante Torpedo

% Messgeber
K_MESS = 1; % [-] Verstärkung Messgeber 
T_MESS = 0.5; % [s] Zeitkonstante Messgeber

% Kursregler (PID)
K_R = 1.0; % [-] Verstärkung Regler 
T_D = T_Nomoto; % [s] Zeitkonstante D-Anteil
T_DV = 0.1;  % [s] Zeitkonstante T1-Anteil 
T_I = 200; % [s] Zeitkonstante I Anteil 

% -- Modell Erstellung -- %
% Rudermaschine
G_RM_N = [K_RM]; % Nominator
G_RM_D = [T_RM 1]; % Denuminator
G_RM = tf([K_RM], [T_RM 1]); % übertragungsfunktion

% Kursverhalten
G_TRPD_N = [K_IS]; % Nominator
G_TRPD_D = [T_Nomoto 1 0]; % Denuminator
G_TRPD = tf(G_TRPD_N, G_TRPD_D); % übertragungsfunktion

% Messgeber
G_MESS_N = [K_MESS]; % Nominator
G_MESS_D = [T_MESS 1]; % Denumintor
G_MESS = tf(G_MESS_N, G_MESS_D); % übertragungsfunktion

% Kursregler
G_REG_N = K_R * [(T_I * (T_DV + T_D)) (T_DV + T_I) 1];
G_REG_D = [(T_DV * T_I) T_I 0];
G_REG = tf(G_REG_N, G_REG_D);

% Gesamtsystem (Rudermaschine + Torpedo)
G_SYS = series(G_RM, G_TRPD);

%% --- Zustandsraum --- %%

SS_A = [-1 / T_RM 0 0; K_IS / T_Nomoto -1 / T_Nomoto 0; 0 1 0];
SS_B = [K_RM / T_RM; 0; 0];
SS_C = [0 0 1];
SS_D = 0;


% Zustandsmatrizen Messsystem
[SS_A_CNT_MESS, SS_B_CNT_MESS, SS_C_CNT_MESS, SS_D_CNT_MESS] = tf2ss(G_MESS_N, G_MESS_D);

% Wunschpole
sp1 = -0.7;
sp2 = -2.0 + 1.2i;
sp3 = -2.0 - 1.2i;

% Wunschpole Vektor
placed_poles = [sp1 sp2 sp3];

% Reglerparameter
K_R = place(SS_A, SS_B, placed_poles);

%% --- Berechnung --- %%

%sisotool(G_RM * G_TRPD, G_PID, G_MESS)

P_VORFILTER = -(SS_C * ((SS_A - SS_B * K_R)^-1) * SS_B)^-1;
K_I_VORFILTER = 0.1;

while( (sqrt((x_z(k) - x_t(k))^2 + (y_t(k) - y_z(k))^2)>=5) && ((k * dt)<t_max))
    
    % Berechnung Sollwinkel
    phi_soll(k) = atan2((x_z(k) - x_t(k)), (y_t(k) - y_z(k)));

    u_w(k) = P_VORFILTER * phi_soll(k);

    % Berechnung Eingangsgröße Regler

    e_phi(k) = phi_soll(k) - phi_MESS(k);

    u_REG(k) = K_R(1,1) * x1(k) + K_R(1,2) * x2(k) + K_R(1,3) * x3(k);

    u_INT(k + 1) = u_INT(k) + dt * (K_I_VORFILTER * e_phi(k));

    u_GES(k) = u_w(k) - u_REG(k) + u_INT(k);

    x1(k + 1) = x1(k) + dt * (SS_A(1,1) * x1(k) + SS_A(1,2) * x2(k) + SS_A(1,3) * x3(k) + SS_B(1) * u_GES(k)); % Aktuatorgleichung
    x2(k + 1) = x1(k) + dt * (SS_A(2,1) * x1(k) + SS_A(2,2) * x2(k) + SS_A(2,3) * x3(k) + SS_B(2) * u_GES(k)); % Bewegnungsgleichung
    x3(k + 1) = x1(k) + dt * (SS_A(3,1) * x1(k) + SS_A(3,2) * x2(k) + SS_A(3,3) * x3(k) + SS_B(3) * u_GES(k)); % Ausgangsgleichung
    
    phi_ist(k) = SS_C(1,1) * x1(k) + SS_C(1,2) * x2(k) + SS_C(1,3) * x3(k) + SS_D * u_GES(k);

    x1_MESS(k + 1) = x1_MESS(k) + dt * (SS_A_CNT_MESS(1,1) * x1_MESS(k) + SS_B_CNT_MESS(1) * phi_ist(k));

    phi_MESS(k) = SS_C_CNT_MESS(1) * x1_MESS(k) + SS_D_CNT_MESS * phi_ist(k);

    phi_MESS(k + 1) = phi_MESS(k);

    % Berechnung Position Torpedo
    x_t(k + 1) = x_t(k) + v_t * sin(phi_ist(k)) * dt;
    y_t(k + 1) = y_t(k) + v_t * -cos(phi_ist(k)) * dt;
    
    % Zick Zack Kurs Zerstörer -> wenn Torpedo näher als 2000m
    if(sqrt((y_z(k) - y_t(k))^2 + (x_z(k) - x_t(k))^2)<=2000)

        phi_z(k) = phi_flucht + phi_z_zz;

        if mod(timestamp,30)==0
            phi_z_zz = -phi_z_zz;
        end
        
    end

    % Berechnung Position Schiff
    x_z(k + 1) = x_z(k) + dt * v_z * cos(phi_z(k));
    y_z(k + 1) = y_z(k) + dt * v_z * -sin(phi_z(k));
   
    % Zeitstempel speichern
    timestamp = (k - 1) * dt;
    time_vec(k) = timestamp;
    
    % Laufvariable erhöhen
    k = k + 1;
end

%% Darstellung Laufbahnen
figure('Name', 'Seekarte', 'Position', [5 5 800 1000])
plot (x_t(1:k),y_t(1:k),'rs', 'MarkerSize', 3)
hold on
plot (x_z(1:k),y_z(1:k),'bd', 'MarkerSize', 3)
legend('Laufbahn Torpedo','Laufbahn Zerstörer')
hold off
grid on
axis equal
axis([-2000 4000 -3000 4000])


%% Darstellung Laufbahn in "Echtzeit"
plotRealTime(sim_length, x_t,y_t,x_z,y_z)

% functions declaration
function plotRealTime(sim_length, x_t, y_t, x_z, y_z)
    xtData = zeros(1, sim_length);
    ytData = zeros(1, sim_length);
    xzData = zeros(1, sim_length);
    yzData = zeros(1, sim_length);

    xtData(1) = x_t(1);
    ytData(1) = y_t(1);
    xzData(1) = x_z(1);
    yzData(1) = y_z(1);

    figure('Name', 'Seekarte Echtzeit', 'Position', [1000 5 800 1000])
    hold on
    tplot = plot (xtData(1), ytData(1), 'rs', 'MarkerSize', 3);
    zplot = plot (xzData(1), yzData(1), 'bd', 'MarkerSize', 3);
    legend('Laufbahn Torpedo','Laufbahn Zerstörer')
    grid on
    axis equal
    axis([-2000 4000 -3000 4000])
    i = 1;
    j = 2;
    while (sqrt((x_z(i) - x_t(i))^2 + (y_t(i) - y_z(i))^2)>=5)
        if mod(i, 5000) == 0
            xtData(j) = x_t(i);
            ytData(j) = y_t(i);
            xzData(j) = x_z(i);
            yzData(j) = y_z(i);

            set(tplot, 'XData', xtData(1:i), 'YData', ytData(1:i))
            set(zplot, 'XData', xzData(1:i), 'YData', yzData(1:i))
            j = j + 1;
        end
        i = i + 1;
        pause(0.00001)
    end

    hold off
end

