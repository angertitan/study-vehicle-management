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

%Gesamtübertragungsfunktion Rudermaschine + Torpedo
G_gesamt = series(G_RM,G_torpedo);

%Zustandsraummatrizen
ss_A_Strecke = [-1/T_RM 0 0; K_IS/T_NOMOTO -1/T_NOMOTO 0; 0 1 0];
ss_B_Strecke = [K_RM/T_RM; 0; 0];
ss_C_Strecke = [0 0 1];
ss_D_Strecke = 0;

%Überführung des Messsystems in Zustandsmatrizen
[G_MESS_num,G_MESS_den] = tfdata(G_MESS,'V');                   %Speicherung Zähler und Nenner Übertragungsfkt. in Vektoren
[SS_A_CNT_MESS,SS_B_CNT_MESS,SS_C_CNT_MESS,SS_D_CNT_MESS] = tf2ss(G_MESS_num,G_MESS_den);             %Überführung in Matrizen Zustandsraum

%Wunschpole
sp1 = -0.7;
sp2 = -2.0 +1.2i;   
sp3 = -2.0 - 1.2i;

%Vektor der Wunschpole
Placed_Poles = [sp1 sp2 sp3];

%Reglerparameter 
K_r = place(ss_A_Strecke,ss_B_Strecke,Placed_Poles);

%Berechnung des Vorfilters zum Abgleich Soll-Ist-Wert
P_VORFILTER = -(ss_C_Strecke*((ss_A_Strecke-ss_B_Strecke*K_r)^-1)*ss_B_Strecke)^-1;
K_I_VORFILTER = 0.1; 

%Vektoren
vec_u_w = zeros(1,length_sim);
vec_u_REG = zeros(1,length_sim);
vec_u_INT = zeros(1,length_sim);
vec_u_ges = zeros(1,length_sim);
vec_e_phi = zeros(1,length_sim);
vec_x1 = zeros(1,length_sim);
vec_x2 = zeros(1,length_sim);
vec_x3 = zeros(1,length_sim);
vec_phi_ist= zeros(1,length_sim);
vec_x1_mess_ist = zeros(1,length_sim);
vec_phi_mess_ist = zeros(1,length_sim);

%Simulation
while ((sqrt((vec_XT(k) - vec_XS(k))^2 + (vec_YT(k) - vec_YS(k))^2)) > 5) && (vec_t(k) < T_max) %Bedinungen der while-Schleife:solange Feindradius und Detonationszeit nicht erreicht ist
    
    vec_phi_soll(k) = atan2((vec_XS(k) - vec_XT(k)),(vec_YT(k) - vec_YS(k)));    %Vorgabe Winkel Torpedo
    
    vec_u_w(k) = P_VORFILTER*vec_phi_soll(k);
    
    %Berechnung Eingangsgröße Regler
    vec_e_phi(k) = vec_phi_soll(k) - vec_phi_mess_ist(k);
    
    vec_u_REG(k) = K_r(1,1)*vec_x1(k) + K_r(1,2)*vec_x2(k) + K_r(1,3)*vec_x3(k);
    
    vec_u_INT(k+1) = vec_u_INT(k) + delta_t*(K_I_VORFILTER*vec_e_phi(k));
    
    vec_u_ges(k) = vec_u_w(k) - vec_u_REG(k) + vec_u_INT(k);
    
    vec_x1(k+1) = vec_x1(k) + delta_t*(ss_A_Strecke(1,1)*vec_x1(k) + ss_A_Strecke(1,2)*vec_x2(k) + ss_A_Strecke(1,3)*vec_x3(k) + ss_B_Strecke(1)*vec_u_ges(k)); %Aktuatorgleichung
    vec_x2(k+1) = vec_x2(k) + delta_t*(ss_A_Strecke(2,1)*vec_x1(k) + ss_A_Strecke(2,2)*vec_x2(k) + ss_A_Strecke(2,3)*vec_x3(k) + ss_B_Strecke(2)*vec_u_ges(k));%Bewegungsgleichung ROT
    vec_x3(k+1) = vec_x3(k) + delta_t*(ss_A_Strecke(3,1)*vec_x1(k) + ss_A_Strecke(3,2)*vec_x2(k) + ss_A_Strecke(3,3)*vec_x3(k) + ss_B_Strecke(3)*vec_u_ges(k));% Ausgangsgleichung Integration ROT --> phi_ist
    
    vec_phi_ist(k) = ss_C_Strecke(1,1)*vec_x1(k) + ss_C_Strecke(1,2)*vec_x2(k) + ss_C_Strecke(1,3)*vec_x3(k) + ss_D_Strecke*vec_u_ges(k);
    
    vec_x1_mess_ist(k+1) = vec_x1_mess_ist(k) + delta_t*(SS_A_CNT_MESS(1,1)*vec_x1_mess_ist(k) + SS_B_CNT_MESS(1)*vec_phi_ist(k));
    
    vec_phi_mess_ist(k) = SS_C_CNT_MESS(1)*vec_x1_mess_ist(k) +  SS_D_CNT_MESS*vec_phi_ist(k);
    
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
