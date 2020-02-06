%Initialisierung

delta_t = 0.001;                                        %delta_t für EULER-Simulationsalgorhtithmus
T_max = 500;                                            %Detonationszeit
Y_Uboot_0 = 4000;                                       %Position Uboot
length_sim = T_max / delta_t;                           %Simulationslänge
vec_XT = zeros(1,length_sim);                           %X-Position Torpedo
vec_YT = zeros(1,length_sim);                           %Y-Position Torpedo
vec_XS = zeros(1,length_sim);                           %X-Position Schiff
vec_YS = zeros(1,length_sim);                           %Y-Position Schiff
vec_phi_soll = zeros(1,length_sim);                     %Einstellwinkel Torpedoruder
vec_phi_ist = zeros(1,length_sim);                      %Aktueller Winkel Torpedoruder
phi_s = 0;                                              %Fahrtwinkel Schiff
vec_t = zeros(1,length_sim);                            %Zeitgröße für die Simulation
vec_YT(1) = Y_Uboot_0;                                  %Anfangsposition Torpedo
vt = 50;                                                %Topedogeschwindigkeit in Knoten
vs = 30;                                                %Schiffgeschwindigkeit in Knoten
kn_zu_ms = 1852/3600;                                   %Umrechnungsfaktor Knoten in Meter pro Sekunde
vT = vt * kn_zu_ms;                                     %Torpedogeschwindigkeit in m/s
vS = vs * kn_zu_ms;                                     %Schiffgeschwindigkeit in m/s
k = 1;                                                  %Variable für das Weiterrücken im Vektor
phi_zickzack = 30*(pi/180);                             %Fahrtwinkel Schiff für Zickzack
phi_gen_flucht = -45*(pi/180);                          %Fahrtwinkel Schiff nach Süden
vec_rw_ist = zeros(1,length_sim);                       %Vektor für den Ruderwinkel in rad
vec_rw_ist_grad = zeros(1,length_sim);                  %Vektor für den Ruderwinkel in Grad

%PT1-Parameter Torpedo
Kp = 1;                                                 %Verstärkung
T_torp = 17;                                            %Verzögerungszeitkonstante in Sekunden

%Parameter Regelstrecke
K_IS = 1.0;                                             %Verstärkung Torpedo
K_RM = 1.0;                                             %Verstärkung Rudermaschine
K_MESS = 1.0;                                           %Verstärkung Messsystem
T_NOMOTO =4.0;                                          %Zeitkonstante Torpedo
T_MESS = 0.5;                                           %Zeitkonstante Messsystem
T_RM = 0.2;                                             %Zeitkonstante Rudermaschine

%Paramter Regler
K_REG = 1.0;                                            %Reglerverstärkung
T_D = T_NOMOTO;                                         %Zeitkonstante D-Anteil Regler
T_DV = 0.1;                                             %Zeitkonstante T1-Anteil Regler
T_I = 200;                                              %Zeitkonstante I-Anteil Regler

%Reglerstrecke
G_RM = tf(K_RM,[T_RM 1]);                               %Übertragungsfunktion Rudermaschine
G_MESS = tf(K_MESS,[T_MESS 1]);                         %Übertragungsfunktion Messeinrichtung
G_torpedo = tf(K_IS,[T_NOMOTO 1]) * tf(1,[1 0]);        %Übertragungsfunktion Torpedo

%Regler
G_REGLER1 = tf(K_REG,1)+tf(K_REG*[T_D 0],[T_DV 1]) + tf(K_REG,[T_I 0]);     %Übertragungsfunktion Regler 1
G_REGLER2 = tf(K_REG*[T_I*(T_D + T_DV) (T_I + T_DV) 1],[T_I*T_DV T_I 0]);   %Übertragungsfunktion Regler 2

%Gesamtsystem für Torpedowinkel
G_PHI_I_PHI_S = feedback(series(G_REGLER2,series(G_RM,G_torpedo)),G_MESS);  %Gesamtübertragungsfunktion

%figure
%step(G_PHI_I_PHI_S,40)

[G_PHI_I_PHI_S_num,G_PHI_I_PHI_S_den] = tfdata(G_PHI_I_PHI_S,'v');

poles_closed_loop = roots(G_PHI_I_PHI_S_den)

%Regelstrecke im Zustandsraum
ss_A = [-1/T_RM 0 0; K_IS/T_NOMOTO -1/T_NOMOTO 0; 0 1 0];
ss_B = [K_RM/T_RM; 0; 0];
ss_C = [0 0 1];
ss_D = 0;

%Bestimmung der gegebenen Prozesspole (Regelstreckenpolstellen)
proc_eig = eig(ss_A)

%Wunschpole
sp1 = -0.7;
sp2 = -2.0 + 1.2i;   
sp3 = -2.0 - 1.2i;

%Vektor der Wunschpole
Placed_Poles = [sp1 sp2 sp3]

%Ausmultiplizieren --> charakteristische Gleichung
char_poly = conv([1 -sp1],conv([1 -sp2],[1 -sp3]))

%Probe --> müssen wieder Wunschpole rauskommen
roots(char_poly)

K_r = place(ss_A,ss_B,Placed_Poles)

%Berechnung des Vorfilters zum Abgleich Soll-Ist-Wert
P_VORFILTER = -(ss_C*((ss_A-ss_B*K_r)^-1)*ss_B)^-1