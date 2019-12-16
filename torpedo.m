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
v_z = 40 * kn2mjs; % [m/s] Geschwindigkeit Zerstörer (Knoten -> m/s)
phi_z_zz = deg2rad(20); % [rad] Zick Zack Winkel der Zerstörers

%% --- Vektoren --- %%
% -- Simulations Vektoren -- %
base_vec = zeros(1, sim_length); % Basisvektor
time_vec = zeros(1, sim_length); % [s] Zeitvektor

% --Torpedo Vektoren -- %
x1 = base_vec; % Zustandsvektoren
x2 = base_vec; % Zustandsvektoren
x3 = base_vec; % Zustandsvektoren
x4 = base_vec; % Zustandsvektoren
x5 = base_vec; % Zustandsvektoren
x6 = base_vec; % Zustandsvektoren

x_t = base_vec; % [m] X-Richtungsvektor Torpedo
y_t = base_vec; % [m] Y-Richtungsvektor Torpedo
y_t(1) = y_t__start; % [m] Startentfernung setzen

phi_soll = base_vec; % [rad] Sollwinkel Torpedo
phi_ist = base_vec; % [rad] Istwinkel Torpedo

% -- Zerstörer Vektoren -- %
x_z = base_vec; % [m] X-Richtungsvektor Zerstörer
y_z = base_vec; % [m] Y-Richtungsvektor Zerstörer

phi_z = base_vec; % [rad] Winkel Zerstörer

%% --- Modellbildung --- %%
% -- Modell Parameter -- %
% Rudermaschine
K_RM = 1; % [-] Verstärkung Rudermaschine 
T_RM = 0.2; % [s] Zeitkonstante Rudermaschine

% Kursverhalten
K_IS = 1; % [-] Verstärkung Kursverhalten 
T_Nomoto = 4; % [s] Zeitkonstante Kursverhalten

% Messgeber
K_MESS = 1; % [-] Verstärkung Messgeber 
T_MESS = 0.5; % [s] Zeitkonstante Messgeber

% Kursregler (PID)
T_J = 200; % [s] Zeitkonstante I-System Regler 
K_R = 1; % [-] Verstärkung Regler 
T_D = T_Nomoto; % [s] Zeitkonstante 1 PD-System Regler 
T_DV = 0.1;  % [s] Zeitkonstante 2 PD-System Regler 

% -- Modell Erstellung -- %
% Rudermaschine
G_RM_N = [K_RM]; % Nominator
G_RM_D = [T_RM 1]; % Denuminator
G_RM = tf(G_RM_N, G_RM_D); % übertragungsfunktion

% Kursverhalten
G_PHI_N = [K_IS]; % Nominator
G_PHI_D = [T_Nomoto 1 0]; % Denuminator
G_PHI = tf(G_PHI_N, G_PHI_D); % übertragungsfunktion

% Messgeber
G_MESS_N = [K_MESS]; % Nominator
G_MESS_D = [T_MESS 1]; % Denumintor
G_MESS = tf(G_MESS_N, G_MESS_D); % übertragungsfunktion

% Kursregler
G_PID_N = K_R * [(T_J * (T_DV + T_D)) (T_DV + T_J) 1];
G_PID_D = [(T_DV * T_J) T_J 0];
G_PID = tf(G_PID_N, G_PID_D);

% Gesamtsystem
CLOOP_T = feedback(series(series(G_PID, G_RM), G_PHI), G_MESS);

step(CLOOP_T, 40)
%% --- Zustandsraum --- %%

[CLOOP_T__N, CLOOP_T__D] = tfdata(CLOOP_T, 'v');

% Regelungsnormalform
[SS_A_CNT, SS_B_CNT, SS_C_CNT, SS_D_CNT] = tf2ss(CLOOP_T__N, CLOOP_T__D);

% Beobachternormalform

SS_A_OBS = SS_A_CNT';
SS_B_OBS = SS_C_CNT';
SS_C_OBS = SS_B_CNT';
SS_D_OBS = SS_D_CNT;

SS_A = SS_A_CNT;
SS_B = SS_B_CNT;
SS_C = SS_C_CNT;
SS_D = SS_D_CNT;
%% --- Berechnung --- %%

%sisotool(G_RM * G_PHI, G_PID, G_MESS)

while( (sqrt((x_z(k) - x_t(k))^2 + (y_t(k) - y_z(k))^2)>=5) && ((k * dt)<t_max))
    
    % Berechnung Sollwinkel
    phi_soll(k) = atan2((x_z(k) - x_t(k)), (y_t(k) - y_z(k)));

    % Berechnung Zustände
    x1(k + 1) = x1(k) + dt * (SS_A(1,1) * x1(k)+(SS_A(1,2)) * x2(k) + (SS_A(1,3)) * x3(k) + (SS_A(1,4)) * x4(k)+(SS_A(1,5)) * x5(k) + (SS_A(1,6)) * x6(k) + SS_B(1) * phi_soll(k));
    x2(k + 1) = x2(k) + dt * (SS_A(2,1) * x1(k)+(SS_A(2,2)) * x2(k) + (SS_A(2,3)) * x3(k) + (SS_A(2,4)) * x4(k)+(SS_A(2,5)) * x5(k) + (SS_A(2,6)) * x6(k) + SS_B(2) * phi_soll(k));
    x3(k + 1) = x3(k) + dt * (SS_A(3,1) * x1(k)+(SS_A(3,2)) * x2(k) + (SS_A(3,3)) * x3(k) + (SS_A(3,4)) * x4(k)+(SS_A(3,5)) * x5(k) + (SS_A(3,6)) * x6(k) + SS_B(3) * phi_soll(k));
    x4(k + 1) = x4(k) + dt * (SS_A(4,1) * x1(k)+(SS_A(4,2)) * x2(k) + (SS_A(4,3)) * x3(k) + (SS_A(4,4)) * x4(k)+(SS_A(4,5)) * x5(k) + (SS_A(4,6)) * x6(k) + SS_B(4) * phi_soll(k));
    x5(k + 1) = x5(k) + dt * (SS_A(5,1) * x1(k)+(SS_A(5,2)) * x2(k) + (SS_A(5,3)) * x3(k) + (SS_A(5,4)) * x4(k)+(SS_A(5,5)) * x5(k) + (SS_A(5,6)) * x6(k) + SS_B(5) * phi_soll(k));
    x6(k + 1) = x6(k) + dt * (SS_A(6,1) * x1(k)+(SS_A(6,2)) * x2(k) + (SS_A(6,3)) * x3(k) + (SS_A(6,4)) * x4(k)+(SS_A(6,5)) * x5(k) + (SS_A(6,6)) * x6(k) + SS_B(6) * phi_soll(k));
    
    % Berechnung Istwinkel
    phi_ist(k) = SS_C(1) * x1(k) + SS_C(2) * x2(k) + SS_C(3) * x3(k) + SS_C(4) * x4(k) + SS_C(5) * x5(k) + SS_C(6) * x6(k);

    phi_ist(k + 1) = phi_ist(k) + dt * ( -(1/T_t) * phi_ist(k) + (K_p/T_t) * phi_soll(k));
    
    % Berechnung Position Torpedo
    x_t(k + 1) = x_t(k) + v_t * sin(phi_ist(k)) * dt;
    y_t(k + 1) = y_t(k) + v_t * -cos(phi_ist(k)) * dt;
    
    % Zick Zack Kurs Zerstörer -> wenn Torpedo näher als 2000m
    if(sqrt((y_z(k) - y_t(k))^2 + (x_z(k) - x_t(k))^2)<=2000)
        
        phi_z(k) = phi_z(k) + phi_z_zz;

        if mod(timestamp,15)==0
            phi_z_zz = -phi_z_zz;
        end
      
    end

    % Berechnung Position Schiff
    x_z(k + 1) = x_z(k) + v_z * cos(phi_z(k)) * dt;
    y_z(k + 1) = y_z(k) + v_z * -sin(phi_z(k)) * dt;
   

    % Zeitstempel speichern
    timestamp = (k - 1) * dt;
    time_vec(k) = timestamp;
    
    % Laufvariable erhöhen
    k = k + 1;
end

%% Darstellung und Anzeige
figure()
plot (x_t(1:k),y_t(1:k),'r')
hold on
plot (x_z(1:k),y_z(1:k),'b')
legend('Laufbahn Torpedo','Laufbahn Zerstörer')
