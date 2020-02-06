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
phi_s = 0;                                                          %Fahrtwinkel Schiff
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

%PT1-Parameter Torpedo
Kp = 1;                                                 %Verstärkung
T_torp = 17;                                            %Verzögerungszeitkonstante in Sekunden

K_REG = 1.0;        %Reglerverstärkung
K_IS = 1.0;         %Verstärkung Torpedo
K_RM = 1.0;         %Verstärkung Rudermaschine
K_MESS = 1.0;       %Verstärkung Messsystem
T_NOMOTO =4.0;      %Zeitkonstante Torpedo
T_MESS = 0.5;       %Zeitkonstante Messsystem
T_RM = 0.2;         %Zeitkonstante Rudermaschine
T_D = T_NOMOTO;     %Zeitkonstante D-Anteil Regler
T_DV = 0.1;         %Zeitkonstante T1-Anteil Regler
T_I = 200;          %Zeitkonstante I-Anteil Regler

%Reglerstrecke
G_RM = tf(K_RM,[T_RM 1]);                               %Übertragungsfunktion Rudermaschine
G_MESS = tf(K_MESS,[T_MESS 1]);                         %Übertragungsfunktion Messeinrichtung
G_torpedo = tf(K_IS,[T_NOMOTO 1]) * tf(1,[1 0]);        %Übertragungsfunktion Torpedo

%Regler
G_REGLER1 = tf(K_REG,1)+tf(K_REG*[T_D 0],[T_DV 1]) + tf(K_REG,[T_I 0]);
G_REGLER2 = tf(K_REG*[T_I*(T_D + T_DV) (T_I + T_DV) 1],[T_I*T_DV T_I 0]);

%Gesamtsystem
G_PHI_I_PHI_S = feedback(series(G_REGLER2,series(G_RM,G_torpedo)),G_MESS);   %Gesamtübertragungsfunktion

%Überführung in den Zustandsraum
[G_PHI_I_PHI_S_num,G_PHI_I_PHI_S_den]=tfdata(G_PHI_I_PHI_S,'V');
[SS_A_CNT,SS_B_CNT,SS_C_CNT,SS_D_CNT]=tf2ss(G_PHI_I_PHI_S_num,G_PHI_I_PHI_S_den);

%sisotool(series(G_RM,G_torpedo),G_REGLER2,G_MESS,tf(1,1))

%Vektoren Zustandsraum
vec_x1 = zeros(1,length_sim); 
vec_x2 = zeros(1,length_sim); 
vec_x3 = zeros(1,length_sim); 
vec_x4 = zeros(1,length_sim); 
vec_x5 = zeros(1,length_sim); 
vec_x6 = zeros(1,length_sim); 

%Simulation
while ((sqrt((vec_XT(k) - vec_XS(k))^2 + (vec_YT(k) - vec_YS(k))^2)) > 5) && (vec_t(k) < T_max) %Bedinungen der while-Schleife:solange Feindradius und Detonationszeit nicht erreicht ist
    
    vec_phi_soll(k) = atan2((vec_XS(k) - vec_XT(k)),(vec_YT(k) - vec_YS(k)));    %Vorgabe Ruderwinkel Torpedo
    %vec_phi_ist(k+1) = vec_phi_ist(k) + delta_t*(-1/T_torp*vec_phi_ist(k)+Kp/T_torp*vec_phi_soll(k));   %Einstellung Ruderwinkel Torpedo  
    
    vec_x1(k+1) = vec_x1(k) + delta_t * (SS_A_CNT(1,1)*vec_x1(k) + SS_A_CNT(1,2)*vec_x2(k) + SS_A_CNT(1,3)*vec_x3(k) + SS_A_CNT(1,4)*vec_x4(k) + SS_A_CNT(1,5)*vec_x5(k) + SS_A_CNT(1,6)*vec_x6(k) + SS_B_CNT(1)*vec_phi_soll(k));
    vec_x2(k+1) = vec_x2(k) + delta_t * (SS_A_CNT(2,1)*vec_x1(k) + SS_A_CNT(2,2)*vec_x2(k) + SS_A_CNT(2,3)*vec_x3(k) + SS_A_CNT(2,4)*vec_x4(k) + SS_A_CNT(2,5)*vec_x5(k) + SS_A_CNT(2,6)*vec_x6(k) + SS_B_CNT(2)*vec_phi_soll(k));
    vec_x3(k+1) = vec_x3(k) + delta_t * (SS_A_CNT(3,1)*vec_x1(k) + SS_A_CNT(3,2)*vec_x2(k) + SS_A_CNT(3,3)*vec_x3(k) + SS_A_CNT(3,4)*vec_x4(k) + SS_A_CNT(3,5)*vec_x5(k) + SS_A_CNT(3,6)*vec_x6(k) + SS_B_CNT(3)*vec_phi_soll(k));
    vec_x4(k+1) = vec_x4(k) + delta_t * (SS_A_CNT(4,1)*vec_x1(k) + SS_A_CNT(4,2)*vec_x2(k) + SS_A_CNT(4,3)*vec_x3(k) + SS_A_CNT(4,4)*vec_x4(k) + SS_A_CNT(4,5)*vec_x5(k) + SS_A_CNT(4,6)*vec_x6(k) + SS_B_CNT(4)*vec_phi_soll(k));
    vec_x5(k+1) = vec_x5(k) + delta_t * (SS_A_CNT(5,1)*vec_x1(k) + SS_A_CNT(5,2)*vec_x2(k) + SS_A_CNT(5,3)*vec_x3(k) + SS_A_CNT(5,4)*vec_x4(k) + SS_A_CNT(5,5)*vec_x5(k) + SS_A_CNT(5,6)*vec_x6(k) + SS_B_CNT(5)*vec_phi_soll(k));
    vec_x6(k+1) = vec_x6(k) + delta_t * (SS_A_CNT(6,1)*vec_x1(k) + SS_A_CNT(6,2)*vec_x2(k) + SS_A_CNT(6,3)*vec_x3(k) + SS_A_CNT(6,4)*vec_x4(k) + SS_A_CNT(6,5)*vec_x5(k) + SS_A_CNT(6,6)*vec_x6(k) + SS_B_CNT(6)*vec_phi_soll(k));
  
    vec_phi_ist(k) = SS_C_CNT(1)*vec_x1(k) +  SS_C_CNT(2)*vec_x2(k) + SS_C_CNT(3)*vec_x3(k) + SS_C_CNT(3)*vec_x3(k) + SS_C_CNT(4)*vec_x4(k) + SS_C_CNT(5)*vec_x5(k) + SS_C_CNT(6)*vec_x6(k) + SS_D_CNT * vec_phi_soll(k);
    
    
    vec_XT(k+1) = vec_XT(k) + delta_t * (vT * sin(vec_phi_ist(k)));     %Berechnung neuer X-Position Torpedo
    vec_YT(k+1) = vec_YT(k) + delta_t * (-vT * cos(vec_phi_ist(k)));    %Berechnung neuer Y-Position Torpedo
    
    if sqrt((vec_XT(k) - vec_XS(k))^2 + (vec_YT(k) - vec_YS(k))^2) > 2000 %normale Fahrt bei über 2000m Abstand
        
        vec_XS(k+1) = vec_XS(k) + delta_t * (vS * cos(phi_s));  %Berechnung neuer X-Position Schiff
        vec_YS(k+1) = vec_YS(k) + delta_t * (vS * sin(phi_s));  %Berechnung neuer Y-Position Schiff
    
    else %Ausweichen
        
        vec_XS(k+1) = vec_XS(k) + delta_t * (vS * cos(phi_zickzack+phi_gen_flucht));  %Berechnung neuer X-Position Schiff zum Ausweichen
        vec_YS(k+1) = vec_YS(k) + delta_t * (vS * sin(phi_zickzack+phi_gen_flucht));  %Berechnung neuer Y-Position Schiff zum Ausweichen        
    
        if mod(vec_t(k),30)==0      %Alle 30 Sekunden Kurs ändern
            phi_zickzack = -phi_zickzack;       %Umkehrung des Winkels von pos. zu neg.
        end
        
    end
    
    vec_t(k+1) = (k)*delta_t;  %Größe mit der die Detonationszeit verglichen wird
    
    k = k+1; %Erhöhung der Variable um zur nächsten Stelle des Vektors zu kommen
    
end

figure
plot(vec_XT(1:k),vec_YT(1:k)) %Darstellung Ubootposition
hold on
plot(vec_XS(1:k),vec_YS(1:k)) %Darstellung Schiffsposition
hold off
grid on     %Gitter aktivieren
axis equal      %Achsen gleich