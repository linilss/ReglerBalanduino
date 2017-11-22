clear all; clc

% Parametrar for Balanduinoroboten
Balanduino.m  = 0.38;      % [kg]  massa for batteri och oversta hyllan med skruvar
Balanduino.M  = 0.97;      % [kg]  massan for resten av robotan
Balanduino.d  = 0.18;      % [m]   avstand mellan hjulaxel och batteriets masscentrum
Balanduino.r  = 0.049;     % [m]   hjulets radie
Balanduino.Ra = 2.4;       % [Ohm] inre resistansen for DC-motorn
Balanduino.La = 0.00025;   % [H]   inre induktansen for DC-motorn
Balanduino.Km = 0.155;     % [-]   omvandlingsfaktor mellan strom och moment 
Balanduino.Ku = 0.3078;    % [-]   omvandlingsfaktor for DC-motorns mot EMK 
Balanduino.g  = 9.81;      % [m/s^2] 

n  = 0;

% Max/min spanning till motorerna
Balanduino.u_max =  12; % [V]
Balanduino.u_min = -12; % [V]

% Definiera Laplace s
s = tf('s');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definiera tillstandsmodellen
% z_dot = Az + Bu
% y = Cz + Du
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Ni skall satta in era framraknade varden i tillstandsmodellen

tmp21 = (((Balanduino.g*(Balanduino.M+Balanduino.m))/(Balanduino.d*Balanduino.M))); 
tmp24 = ((Balanduino.Km)/(Balanduino.r*Balanduino.d*Balanduino.M));
tmp31 = (Balanduino.g*Balanduino.m)/Balanduino.M;
tmp34 = Balanduino.Km/(Balanduino.r * Balanduino.M);
tmp43 = -Balanduino.Ku/(Balanduino.r*Balanduino.La);
tmp44 = -Balanduino.Ra/Balanduino.La;
% A-matrisen
A = [0 1 0 0;
     tmp21 0 0 tmp24;
     tmp31 0 0 tmp34;
     0 0 tmp43 tmp44];
 
 
% B-vektorn
B = [0;
     0;
     0;
     1];

% C-vektorn 
C = [1 0 0 0];

% D-termen (ar noll)
D = 0;

% Definiera den linjara tillstandsmodellen
Balanduino_sys = ss(A,B,C,D)

% Bestam overforningsfunktionen med funktionen "tf"
G = tf(Balanduino_sys)

% Ekvivalent satt att bestamma G(s)
%G = C*inv(s*eye(4)-A)*B

% Bestam polerna 
poler_G = pole(G)
