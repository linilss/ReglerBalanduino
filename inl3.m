
Kp = 28.53;
Ki = 241.53;
Kd = 2.48;
Tf = 1/25;
a = 7.55;
b = 75.85;
g = 9.82;
d = 0.18;

s = tf('s');

F = Kp + Ki/s + Kd*s/(1+Tf*s);
Gv = (d*s^2 - g) / s;
Gpp = F *a /(a*F+s^2-b);
Gp = -Gpp *Gv *1/s

[mag, phase, wout] = bode(Gp,0.4)

frsp = evalfr(Gp,0.4j);
abs(frsp);

bode(Gp);

%%

Kvp = 5.73;
Kvi = 31.25;

Fv = Kvp + Kvi/s;
nyquist(Gp);