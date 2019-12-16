clear all
close all

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

%Überführung des Reglers in Zustandsmatrizen
[G_REG_num,G_REG_den] = tfdata(G_REGLER2,'V');            %Speicherung Zähler und Nenner Übertragungsfkt. in Vektoren
[SS_A_CNT_REG,SS_B_CNT_REG,SS_C_CNT_REG,SS_D_CNT_REG] = tf2ss(G_REG_num,G_REG_den);       %Überführung in Matrizen Zustandsraum

%Überführung der Rudermaschine in Zustandsmatrizen
[G_RM_num,G_RM_den] = tfdata(G_RM,'V');                   %Speicherung Zähler und Nenner Übertragungsfkt. in Vektoren
[SS_A_CNT_RM,SS_B_CNT_RM,SS_C_CNT_RM,SS_D_CNT_RM] = tf2ss(G_RM_num,G_RM_den);             %Überführung in Matrizen Zustandsraum

%Überführung des Torpedos in Zustandsmatrizen
[G_torpedo_num,G_torpedo_den] = tfdata(G_torpedo,'V');                   %Speicherung Zähler und Nenner Übertragungsfkt. in Vektoren
[SS_A_CNT_torpedo,SS_B_CNT_torpedo,SS_C_CNT_torpedo,SS_D_CNT_torpedo] = tf2ss(G_torpedo_num,G_torpedo_den);             %Überführung in Matrizen Zustandsraum

%Überführung des Messsystems in Zustandsmatrizen
[G_MESS_num,G_MESS_den] = tfdata(G_MESS,'V');                   %Speicherung Zähler und Nenner Übertragungsfkt. in Vektoren
[SS_A_CNT_MESS,SS_B_CNT_MESS,SS_C_CNT_MESS,SS_D_CNT_MESS] = tf2ss(G_MESS_num,G_MESS_den);             %Überführung in Matrizen Zustandsraum

%Aufrufen des Control System Designers für G_Phi_I_Phi_S
%sisotool(series(G_RM,G_torpedo),G_REGLER2,G_MESS,tf(1,1))

%Vektoren Zustände
vec_x1_REG = zeros(1,length_sim);                       %Zustand x1_REG
vec_x2_REG = zeros(1,length_sim);                       %Zustand x2_REG
vec_x1_RM = zeros(1,length_sim);                        %Zustand x1_RM
vec_x1_Torp = zeros(1,length_sim);                      %Zustand x1_Torp
vec_x2_Torp= zeros(1,length_sim);                       %Zustand x2_Torp
vec_x1_mess_ist = zeros(1,length_sim);                  %Zustand x1_mess_ist

%Vektor Eingangsgröße
vec_e_phi = zeros(1,length_sim);                        %Eingangsgröße Regler

%Vektoren Ausgangsgrößen
vec_y_R_PIDT1 = zeros(1,length_sim);                    %Ausgangsgröße PIDT1
vec_y_delta_ist = zeros(1,length_sim);                  %Ausgangsgröße Rudermaschine
vec_phi_ist = zeros(1,length_sim);                      %Ausgangsgröße Torpedo
vec_phi_mess_ist = zeros(1,length_sim);                 %Ausgangsgröße Messsystem

%Simulation
while ((sqrt((vec_XT(k) - vec_XS(k))^2 + (vec_YT(k) - vec_YS(k))^2)) > 5) && (vec_t(k) < T_max) %Bedinungen der while-Schleife:solange Feindradius und Detonationszeit nicht erreicht ist
    
    vec_phi_soll(k) = atan2((vec_XS(k) - vec_XT(k)),(vec_YT(k) - vec_YS(k)));    %Vorgabe Winkel Torpedo
    
    %Berechnung Eingangsgröße Regler
    vec_e_phi(k) = vec_phi_soll(k) - vec_phi_mess_ist(k);
    
    %Berechnungen Zustände Regler
    vec_x1_REG(k+1) = vec_x1_REG(k) + delta_t*(SS_A_CNT_REG(1,1)*vec_x1_REG(k) + SS_A_CNT_REG(1,2)*vec_x2_REG(k) + SS_B_CNT_REG(1)*vec_e_phi(k)); 
    vec_x2_REG(k+1) = vec_x2_REG(k) + delta_t*(SS_A_CNT_REG(2,1)*vec_x1_REG(k) + SS_A_CNT_REG(2,2)*vec_x2_REG(k) + SS_B_CNT_REG(2)*vec_e_phi(k));
    
    %Berechnung Ausgang Regler / Eingang Rudermaschine
    vec_y_R_PIDT1(k) = SS_C_CNT_REG(1,1)*vec_x1_REG(k) + SS_C_CNT_REG(1,2)*vec_x2_REG(k) + SS_D_CNT_REG*vec_e_phi(k);
    
    %Berechnung Zustand Rudermaschine
    vec_x1_RM(k+1) = vec_x1_RM(k) + delta_t*(SS_A_CNT_RM(1,1)*vec_x1_RM(k) + SS_B_CNT_RM(1,1)*vec_y_R_PIDT1(k));
    
    %Berechnung der Ausgangsgröße Rudermaschine / Eingangsgröße Torpedo
    vec_y_delta_ist(k) = SS_C_CNT_RM(1,1)*vec_x1_RM(k) + SS_D_CNT_RM*vec_y_R_PIDT1(k);
    
    %Berechung Zustände Torpedo
    vec_x1_Torp(k+1) = vec_x1_Torp(k) + delta_t*(SS_A_CNT_torpedo(1,1)*vec_x1_Torp(k) + SS_A_CNT_torpedo(1,2)*vec_x2_Torp(k) + SS_B_CNT_torpedo(1)*vec_y_delta_ist(k));
    vec_x2_Torp(k+1) = vec_x2_Torp(k) + delta_t*(SS_A_CNT_torpedo(2,1)*vec_x1_Torp(k) + SS_A_CNT_torpedo(2,2)*vec_x2_Torp(k) + SS_B_CNT_torpedo(2)*vec_y_delta_ist(k));
    
    %Berechnung der Ausgangsgröße Torpedo /Eingangsgröße Messsystem
    vec_phi_ist(k) = SS_C_CNT_torpedo(1,1)*vec_x1_Torp(k) + SS_C_CNT_torpedo(1,2)*vec_x2_Torp(k) + SS_D_CNT_torpedo*vec_y_delta_ist(k);
    
    %Berechnung Zustand Messsystem
    vec_x1_mess_ist(k+1) = vec_x1_mess_ist(k) + delta_t*(SS_A_CNT_MESS(1,1)*vec_x1_mess_ist(k) + SS_B_CNT_MESS(1)*vec_phi_ist(k));
    
    %Berechnung der Ausgangsgröße Messsystem
    vec_phi_mess_ist(k) = SS_C_CNT_MESS(1,1)*vec_x1_mess_ist(k) + SS_D_CNT_MESS*vec_phi_ist(k);
    
    %Trick 17
    vec_phi_mess_ist(k+1) = vec_phi_mess_ist(k);
    
    %Berechnung der Torpedoposition
    vec_XT(k+1) = vec_XT(k) + delta_t * (vT * sin(vec_phi_ist(k)));         %Berechnung neuer X-Position Torpedo
    vec_YT(k+1) = vec_YT(k) + delta_t * (-vT * cos(vec_phi_ist(k)));        %Berechnung neuer Y-Position Torpedo
    
    %Berechnung der Schiffsposition
    if sqrt((vec_XT(k) - vec_XS(k))^2 + (vec_YT(k) - vec_YS(k))^2) > 2000   %normale Fahrt bei über 2000m Abstand
        vec_XS(k+1) = vec_XS(k) + delta_t * (vS * cos(phi_s));  %Berechnung neuer X-Position Schiff
        vec_YS(k+1) = vec_YS(k) + delta_t * (vS * sin(phi_s));  %Berechnung neuer Y-Position Schiff
    
    else %Ausweichen
        
        vec_XS(k+1) = vec_XS(k) + delta_t * (vS * cos(phi_zickzack+phi_gen_flucht));  %Berechnung neuer X-Position Schiff zum Ausweichen
        vec_YS(k+1) = vec_YS(k) + delta_t * (vS * sin(phi_zickzack+phi_gen_flucht));  %Berechnung neuer Y-Position Schiff zum Ausweichen        
    
        if mod(vec_t(k),30)==0                          %Alle 30 Sekunden Kurs ändern
            phi_zickzack = -phi_zickzack;               %Umkehrung des Winkels von pos. zu neg.
        end
        
    end
    
    vec_t(k+1) = (k)*delta_t;                           %Größe mit der die Detonationszeit verglichen wird
    
    k = k+1;                                            %Erhöhung der Variable um zur nächsten Stelle des Vektors zu kommen
    
end

figure('Name','Seekarte')
plot(vec_XT(1:k),vec_YT(1:k))                           %Darstellung Ubootposition
hold on
plot(vec_XS(1:k),vec_YS(1:k))                           %Darstellung Schiffsposition
hold off
grid on                                                 %Gitter aktivieren
axis equal                                              %Achsen gleich skalieren
