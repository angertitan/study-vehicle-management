% Aufgabenstellung: Flugregelungen von gesteuerten und ballistischen 
% Flugk�rpersystemen
% (Maschflugk�rper, ballistische Mittelstrecken- und
% Interkontinentalwaffen)

% Auf einer Raketenbasis sind ballistische
% Mittelstreckenwaffen und Interkontinetalwaffen
% stationiert. Derartige Flugk�rper absolvieren mehrere Flugphasen,
% die nachfolgend kurz beschrieben sind: 
% 
% 
% 1.  Start- oder Boost-Phase � 3 bis 5 Minuten, H�he am Ende zwischen 150 
%     und 400 km je nach Flugbahn, Geschwindigkeit typisch 7 km/s 
%     (25.000 km/h), bis zur 1. kosmischen Geschwindigkeit.
% 2.  Mittlere Flugphase � etwa 25 Minuten � suborbitaler Flug in einer 
%     elliptischen Umlaufbahn, deren Apog�um typisch eine H�he von 1.200 km
%     hat. Die gro�e Halbachse dieser Ellipse hat eine L�nge zwischen dem 
%     vollen und dem halben Erdradius; die Projektion der Bahn auf die Erde
%     ist nahe zum Gro�kreis, leicht verschoben wegen der Erdrotation 
%     w�hrend des Flugs. In dieser Phase kann der Flugk�rper mehrere 
%     unabh�ngige Gefechtsk�pfe und Eintrittshilfen wie metallbeschichtete 
%     Folienballons aussto�en, weiterhin Chaff oder ganze T�uschk�rper.
% 3.  Wiedereintrittsphase, beginnend in 100 km H�he � 2 Minuten Dauer � 
%     Einschlag mit einer Geschwindigkeit bis zu 4 km/s (14.400 km/h), 
%     bei fr�hen ICBM weniger als 1 km/s (3.600 km/h).
% 
% Der Startvorgang soll durch ein MATLAB/SIMULINK-gest�tztes Tool simuliert
% werden, dazu soll auch ein Reglersystem entworfen werden, welches die
% Raketenwaffe w�hrend der Startphase stabilisert. Die Stabilisierung
% erfolgt durch die Strahlruder im Abgasmassenstrom des Raketentriebwerkes.
% Bevor das Triebwerk gestartet wird, soll die Rakete jedoch durch die 
% mobile Waffenabschussplattform stabilisiert werden. Dazu sind zun�chst  
% Rakete und deren Plattform als inverses Pendel zu betrachten. Unmittelbar
% nach dem Abheben der Rakete beh�lt sie ihren Charakter eines inversen
% Pendels bei, wobei jedoch die Kraft zur Stabiliserung nicht mehr durch
% die Plattform, sondern durch die Strahlruder im Abgasmassenstrom
% hervorgerufen wird. 
%
% Zur L�sung sind die folgenden Teilaufgaben zu erledigen: 
% 
% 1. Erstellen eines nichtlinearen Zustandsraummodells von Rakete und 
%    Startplattform. 
%    Das Zustandsraummodell soll die folgenden Gr��en beschreiben: 
%    x_w
%    s*x_w 
%    phi 
%    s*phi
%    die Modellierung soll f�r die x-Ebene erfolgen, die r�umliche Regelung
%    wird durch eine genau equivalente Regelung in der y-Ebene realisiert. 
%    
% 2. Abwandeln des Modells f�r die sich in der Startphase befindliche
%    Rakete, also ohne Startplattform. 
%
% 3. Linearisierung des Zustandraummodells und Erstellen einer
%    entsprechenden Zustandsraumdarstellung in MATLAB/SIMULINK. 
%
% 4. Bestimmen der Eigenwerte des Systems
%
% 5. Berechnen eines Zustandsreglers f�r die Stabiliserung der Waffe und
%    zur Einhaltung der Startposition f�r einen m�glichst senkrechten
%    Start. Legen Sie sich dazu eigenst�ndig die Pole des geschlossenen
%    Systems fest. 
%  
% 6. Bestimmen eines Vorfilters und eines I-Anteils zur
%    Sollwertaufschaltung. Nutzen Sie das Separationsprinzip. 
% 
% 7. Aufbau und Test des Gesamtsystems als SIMULINK-Modell. Danach
%    Verfikation des Simulinkmodells durch eine eigene Simulation, durch
%    eine auf dem Eulerprinzip basierende Simulation
% 
% 8. Untersuchung des F�hrungsverhaltens und des St�rausregelungsverhaltens
%    Dazu sind dem System folgende Signale einzuspeisen: 
%    a. Ein Sollwertsprung auf x_w ab 10. sec
%    b. Eine Eingangsst�rung am Prozess von 10 sec Dauer mit einer Kraft
%    von 50 N, 150 N, 500 N ab der 100. sec
%
% 9. Test des nichtlinearen Gesamtsystems als MATLAB-Modell durch
%    eine auf dem Eulerprinzip basierende Simulation
% 
% 10. Zusatz: Entwickeln Sie einen Zustandsbeobachter mit Polvorgabe und
% testen Sie diesen am linearen und nichtlinearen Modell
% 

%---------

m_pendel = 1000;  % [kg]
m_wagen  = 2000;  % [kg]
l_pendel = 2;     % [m]

J = (1/3)*m_pendel*(l_pendel)^2;   % [kg*m^2]

g = 9.81;         % [m*sec^-2]


a23 = -(m_pendel^2*g*l_pendel^2)/((J+m_pendel*l_pendel^2)*(m_wagen+m_pendel)-m_pendel^2*l_pendel^2);

a43 = +(m_pendel*g*l_pendel*(m_wagen+m_pendel))/((J+m_pendel*l_pendel^2)*(m_wagen+m_pendel)-m_pendel^2*l_pendel^2);

b2  = (J+m_pendel*l_pendel^2)/((J+m_pendel*l_pendel^2)*(m_wagen+m_pendel)-m_pendel^2*l_pendel^2);

b4  = -(m_pendel*l_pendel)/((J+m_pendel*l_pendel^2)*(m_wagen+m_pendel)-m_pendel^2*l_pendel^2);


ss_A = [0 1 0 0; 0 0 a23 0; 0 0 0 1; 0 0 a43 0];    %Systemmatrix A: Systemdynamik
ss_B = [0; b2; 0; b4];                              %Systemmatrix B: Steuermatrix
ss_C=[1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];          %Systemmatrix C: alle Zustaende des Prozesses ausgeben 
ss_C1= [1 0 0 0];
n=length(ss_B);
ss_D = zeros(n,1);                                  %Systemmatrix D: Durchgriff 
ss_D1 = 0;

ss_system_pendel_wagen = ss(ss_A,ss_B,ss_C,ss_D)
ss_system_pendel_wagen1 = ss(ss_A,ss_B,ss_C1,ss_D1)


eigenwerte_process = eig(ss_A)
step(ss_system_pendel_wagen,4) 

r = 0.0000001;
vq = [0.001 1 0.001 1];  q = diag(vq);

[ss_k_reg,s_riccati,ee] = lqr(ss_A,ss_B,q,r);

ss_k_reg

s_riccati

ee

ee_1 = eig(ss_A-ss_B*ss_k_reg)

ss_k_reg2 = place(ss_A,ss_B,[-1 -1 -1 -2.2147]) 
