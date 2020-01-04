%% === Reset === %%
clear all
close all

%% === Globale Parameter === %%

% == Konstanten == %
g = 9.81; % [m/s^2] Erdbeschleunigung

% == Simulations Parameter == %%
dt = 0.001; % [s] Abtastrate
t_max = 500; % [s] Maximale Simulationslänge in Sekunden
sim_length = t_max / dt; % [-] maximale Simulationspunkte
i = 1; % [-] Laufvariable
base_vec = zeros(1, sim_length);

% == Missile Parameter == %
m_rakete = 1000; % [kg] Masseschwerpunkt Rakete (Masse Pendel)
m_wagen = 2000; % [kg] Masse Transportwagen
l_rakete = 2; % [m] Länge Rakete

J = (1/3) * m_rakete * (l_rakete)^2; % [kg*m^2] Trägheitsmoment 

%% === Vektoren === %%
x1_xw = base_vec;
x2_pxw = base_vec;
x3_phi = base_vec;
x4_pphi = base_vec

%% === Matrizen === %%

% == Matrizenwerte == %
a1 = -(m_rakete^2 * g * l_rakete^2) / ((J + m_rakete * l_rakete^2) * (m_wagen+m_rakete) - m_rakete^2 * l_rakete^2); % Formel phi x_w**
a2 = (m_rakete * g * l_rakete * (m_wagen+m_rakete)) / ((J + m_rakete * l_rakete^2) * (m_wagen + m_rakete) - m_rakete^2 * l_rakete^2); % Formel phi phi**
b1 = (J + m_rakete * l_rakete^2) / ((J + m_rakete * l_rakete^2) * (m_wagen + m_rakete) - m_rakete^2 * l_rakete^2); % % Formel F_ux x_w**
b2 = -(m_rakete * l_rakete) / ((J + m_rakete * l_rakete^2) * (m_wagen + m_rakete) - m_rakete^2 * l_rakete^2); % Formel F_ux phi**

% == Systemmatrizen == %
ss_a = [0 1 0 0; 0 0 a1 0; 0 0 0 1; 0 0 a2 0]; %Systemmatrix A: Systemdynamik
ss_b = [0; b1; 0; b2]; %Systemmatrix B: Steuermatrix
ss_c = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1]; %Systemmatrix C: alle Zustaende des Prozesses ausgeben 
ss_c1 = [1 0 0 0];

n = length(ss_b);

ss_d = zeros(n, 1); % Systemmatrix D: Durchgriff
ss_d1 = 0;

%% === Zustandsraum === %%

% Wunschpole
pole1 = 1;
pole2 = -1i;
pole3 = -1i;

pole_vector = [pole1 pole2 pole3];

% Reglerparameter
K_R = place(ss_a, ss_b, pole_vector);

%% === Solving === %%
while (k * dt) < t_max
    i = i + 1
end