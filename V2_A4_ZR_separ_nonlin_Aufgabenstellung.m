% Aufgabenstellung: Flugregelungen von gesteuerten und ballistischen 
% Flugkörpersystemen
% (Maschflugkörper, ballistische Mittelstrecken- und
% Interkontinentalwaffen)

% Auf einer Raketenbasis sind ballistische
% Mittelstreckenwaffen und Interkontinetalwaffen
% stationiert. Derartige Flugkörper absolvieren mehrere Flugphasen,
% die nachfolgend kurz beschrieben sind: 
% 
% 
% 1.  Start- oder Boost-Phase – 3 bis 5 Minuten, Höhe am Ende zwischen 150 
%     und 400 km je nach Flugbahn, Geschwindigkeit typisch 7 km/s 
%     (25.000 km/h), bis zur 1. kosmischen Geschwindigkeit.
% 2.  Mittlere Flugphase – etwa 25 Minuten – suborbitaler Flug in einer 
%     elliptischen Umlaufbahn, deren Apogäum typisch eine Höhe von 1.200 km
%     hat. Die große Halbachse dieser Ellipse hat eine Länge zwischen dem 
%     vollen und dem halben Erdradius; die Projektion der Bahn auf die Erde
%     ist nahe zum Großkreis, leicht verschoben wegen der Erdrotation 
%     während des Flugs. In dieser Phase kann der Flugkörper mehrere 
%     unabhängige Gefechtsköpfe und Eintrittshilfen wie metallbeschichtete 
%     Folienballons ausstoßen, weiterhin Chaff oder ganze Täuschkörper.
% 3.  Wiedereintrittsphase, beginnend in 100 km Höhe – 2 Minuten Dauer – 
%     Einschlag mit einer Geschwindigkeit bis zu 4 km/s (14.400 km/h), 
%     bei frühen ICBM weniger als 1 km/s (3.600 km/h).
% 
% Der Startvorgang soll durch ein MATLAB/SIMULINK-gestütztes Tool simuliert
% werden, dazu soll auch ein Reglersystem entworfen werden, welches die
% Raketenwaffe während der Startphase stabilisert. Die Stabilisierung
% erfolgt durch die Strahlruder im Abgasmassenstrom des Raketentriebwerkes.
% Bevor das Triebwerk gestartet wird, soll die Rakete jedoch durch die 
% mobile Waffenabschussplattform stabilisiert werden. Dazu sind zunächst  
% Rakete und deren Plattform als inverses Pendel zu betrachten. Unmittelbar
% nach dem Abheben der Rakete behält sie ihren Charakter eines inversen
% Pendels bei, wobei jedoch die Kraft zur Stabiliserung nicht mehr durch
% die Plattform, sondern durch die Strahlruder im Abgasmassenstrom
% hervorgerufen wird. 
%
% Zur Lösung sind die folgenden Teilaufgaben zu erledigen: 
% 
% 1. Erstellen eines nichtlinearen Zustandsraummodells von Rakete und 
%    Startplattform. 
%    Das Zustandsraummodell soll die folgenden Größen beschreiben: 
%    x_w
%    s*x_w 
%    phi 
%    s*phi
%    die Modellierung soll für die x-Ebene erfolgen, die räumliche Regelung
%    wird durch eine genau equivalente Regelung in der y-Ebene realisiert. 
%    
% 2. Abwandeln des Modells für die sich in der Startphase befindliche
%    Rakete, also ohne Startplattform. 
%
% 3. Linearisierung des Zustandraummodells und Erstellen einer
%    entsprechenden Zustandsraumdarstellung in MATLAB/SIMULINK. 
%
% 4. Bestimmen der Eigenwerte des Systems
%
% 5. Berechnen eines Zustandsreglers für die Stabiliserung der Waffe und
%    zur Einhaltung der Startposition für einen möglichst senkrechten
%    Start. Legen Sie sich dazu eigenständig die Pole des geschlossenen
%    Systems fest. 
%  
% 6. Bestimmen eines Vorfilters und eines I-Anteils zur
%    Sollwertaufschaltung. Nutzen Sie das Separationsprinzip. 
% 
% 7. Aufbau und Test des Gesamtsystems als SIMULINK-Modell. Danach
%    Verfikation des Simulinkmodells durch eine eigene Simulation, durch
%    eine auf dem Eulerprinzip basierende Simulation
% 
% 8. Untersuchung des Führungsverhaltens und des Störausregelungsverhaltens
%    Dazu sind dem System folgende Signale einzuspeisen: 
%    a. Ein Sollwertsprung auf x_w ab 10. sec
%    b. Eine Eingangsstörung am Prozess von 10 sec Dauer mit einer Kraft
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

ss_k_reg2 = place(ss_A,ss_B,ee_1) 
