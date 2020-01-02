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
y_t__start = 4000; % [m] Startentfernung Torpedo <-> Zerstörer

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
y_t(1) = y_t__start; % [m] Startentfernung setzen

phi_soll = base_vec; % [rad] Sollwinkel Torpedo
% phi_t_ist = base_vec; % [rad] Istwinkel Ruder Torpedo (in Rad)
% phi_t_ist__grad = base_vec; % [grad] Istwinkel Ruder Torpedo (in Grad)

% -- Zerstörer Vektoren -- %
x_z = base_vec; % [m] X-Richtungsvektor Zerstörer
y_z = base_vec; % [m] Y-Richtungsvektor Zerstörer

phi_z = base_vec; % [rad] Winkel Zerstörer

% --Zustandsvektoren -- %

% Zustände der einzelnen Modelle
RM_x1 = base_vec; % Zustand X1 Rudermaschine
TRPD_x1 = base_vec; % Zustand X1 Torpedo
TRPD_x2 = base_vec; % Zustand X2 Torpedo
MESS_x1 = base_vec; % Zustand X1 Messsystem
REG_x1 = base_vec; % Zustand X1 Regler
REG_x2 = base_vec; % Zustand X2 Regler

% Eingangsgröße
in_REG = base_vec; % Eingangsgröße Regler (phi)

% Ausgangsgrößen
out_REG = base_vec; % Ausgangsgröße PID (Y_R_PIDT1)
out_RM = base_vec; % Ausgangsgröße Rudermaschine (y_delta_ist)
out_TRPD = base_vec; % [rad] Ausgangsgröße Torpedo (phi)
out_MESS = base_vec; % Ausgangsgröße Messsystem (phi_mess_ist)

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

% Gesamtsystem
G_SYS = feedback(series(series(G_REG, G_RM), G_TRPD), G_MESS);

%% --- Zustandsraum --- %%

% Überführung der Rudermaschine in Zustandsmatrizen
[SS_A_CNT_RM,SS_B_CNT_RM,SS_C_CNT_RM,SS_D_CNT_RM] = tf2ss(G_RM_N, G_RM_D);

% Überführen des Kursverhaltens in Zustandsmatrizen
[SS_A_CNT_TRPD,SS_B_CNT_TRPD,SS_C_CNT_TRPD,SS_D_CNT_TRPD] = tf2ss(G_TRPD_N, G_TRPD_D);

% Überführen des Messsystems in Zustandsmatrizen
[SS_A_CNT_MESS,SS_B_CNT_MESS,SS_C_CNT_MESS,SS_D_CNT_MESS] = tf2ss(G_MESS_N, G_MESS_D);

% Überführung des Reglers in Zustandsmatrizen
[SS_A_CNT_REG,SS_B_CNT_REG,SS_C_CNT_REG,SS_D_CNT_REG] = tf2ss(G_REG_N, G_REG_D);

%% --- Berechnung --- %%

% Aufrufen des Contol System Designers
%sisotool(G_RM * G_PHI, G_PID, G_MESS)

while( (sqrt((x_t(k) - x_z(k))^2 + (y_t(k) - y_z(k))^2)) >= 5) && ((k * dt) < t_max)
    
    % Berechnung Sollwinkel
    phi_soll(k) = atan2((x_z(k) - x_t(k)), (y_t(k) - y_z(k)));

    % Berechnung Eingangsgröße Regler
    in_REG(k) = phi_soll(k) - out_MESS(k);

    % Berechnung Zustände Regler
    REG_x1(k + 1) = REG_x1(k) + dt * (SS_A_CNT_REG(1,1) * REG_x1(k) + SS_A_CNT_REG(1,2) * REG_x2(k) + SS_B_CNT_REG(1) * in_REG(k));
    REG_x2(k + 1) = REG_x2(k) + dt * (SS_A_CNT_REG(2,1) * REG_x1(k) + SS_A_CNT_REG(2,2) * REG_x2(k) + SS_B_CNT_REG(2) * in_REG(k));

    % Berechnung Ausgang Regler
    out_REG(k) = SS_C_CNT_REG(1,1) * REG_x1(k) + SS_C_CNT_REG(1,2) * REG_x2(k) + SS_D_CNT_REG * in_REG(k);

    % Berechnung Zustände Rudermaschine
    RM_x1(k + 1) = RM_x1(k) + dt * (SS_A_CNT_RM(1,1) * RM_x1(k) + SS_B_CNT_RM(1,1) * out_REG(k));

    % Berechnung Ausgangsgröße Rudermaschine
    out_RM(k) = SS_C_CNT_RM(1,1) * RM_x1(k) + SS_D_CNT_RM * out_REG(k);

    % Berechung Zustände Torpedo
    TRPD_x1(k + 1) = TRPD_x1(k) + dt * (SS_A_CNT_TRPD(1,1) * TRPD_x1(k) + SS_A_CNT_TRPD(1,2) * TRPD_x2(k) + SS_B_CNT_TRPD(1) * out_RM(k));
    TRPD_x2(k + 1) = TRPD_x2(k) + dt * (SS_A_CNT_TRPD(2,1) * TRPD_x1(k) + SS_A_CNT_TRPD(2,2) * TRPD_x2(k) + SS_B_CNT_TRPD(2) * out_RM(k));

    % Berechnung Ausgangsgrößen Torpedo
    out_TRPD(k + 1) = SS_C_CNT_TRPD(1,1) * TRPD_x1(k) + SS_C_CNT_TRPD(1,2) * TRPD_x2(k) + SS_D_CNT_TRPD * out_RM(k);

    % Berechnung Zustände Messsystem
    MESS_x1(k + 1) = MESS_x1(k) + dt * (SS_A_CNT_MESS(1,1) * MESS_x1(k) + SS_B_CNT_MESS(1) * out_TRPD(k));

    % Berechnung Ausgangsgrößen Messsystem
    out_MESS(k) = SS_C_CNT_MESS(1,1) * MESS_x1(k) + SS_D_CNT_MESS * out_TRPD(k);

    % Trick 17 ;)
    out_MESS(k+1) = out_MESS(k);

    % Berechnung Position Torpedo (x und y Richtung)
    x_t(k + 1) = x_t(k) + dt * v_t * sin(out_TRPD(k)) ; 
    y_t(k + 1) = y_t(k) + dt * v_t * -cos(out_TRPD(k));
    
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

%% Darstellung und Anzeige
figure('Name', 'Seekarte')
plot (x_t(1:k),y_t(1:k),'r')
hold on
plot (x_z(1:k),y_z(1:k),'b')
legend('Laufbahn Torpedo','Laufbahn Zerstörer')
hold off
grid on
axis equal