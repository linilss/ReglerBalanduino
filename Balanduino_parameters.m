%clear all; clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fysikaliska parametrar för Balanduino roboten. Ändra inte dessa.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Balanduino.m  = 0.38;      % [kg]  massa for batteri och oversta hyllan med skruvar
Balanduino.M  = 0.97;      % [kg]  massan for resten av robotan
Balanduino.d  = 0.18;      % [m]   avstand mellan hjulaxel och batteriets masscentrum
Balanduino.r  = 0.049;     % [m]   hjulets radie
Balanduino.Ra = 2.4;       % [Ohm] inre resistansen for DC-motorn
Balanduino.La = 0.00025;   % [H]   inre induktansen for DC-motorn
Balanduino.Km = 0.155;     % [-]   omvandlingsfaktor mellan strom och moment 
Balanduino.Ku = 0.3078;    % [-]   omvandlingsfaktor for DC-motorns mot-EMK 
Balanduino.g  = 9.81;      % [m/s^2] 
% Max/min spanning till motorerna
Balanduino.u_max =  12; % [V]
Balanduino.u_min = -12; % [V]

% Samplingstid for den diskreta regulatorn
Balanduino.Ts = 0.004; % exekveringstid for programmet

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Regulatorparametrar. Dessa ska ni ändra till de värden ni beräknat.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Inre regulatorn 1: F_theta 
Controllers.F_theta.Kp = 28.53;
Controllers.F_theta.Ki = 241.53;
Controllers.F_theta.Kd = 1.68;
Controllers.F_theta.Tf = 1/25;

% Yttre regulatorn 1: F_v
Controllers.F_v.Kp = -0.0;
Controllers.F_v.Ki = -0.0;


