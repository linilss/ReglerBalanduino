s=tf('s');

K_m = 0.155;
L_A = 0.25 * 10^(-3);
J = 11.5*10^(-4);
R_a = 2.4;
K_u = 0.3078;
b = 0.0025;

G_uw = K_m/(s^(2)*L_A*J+s*(L_A/R_a+J/b)+b*R_a+K_u*K_m)

F_p = 0.5;
F_i = 4.0/s;
F_pi = (0.1*s + 4)/s;

H_p = (F_p*G_uw) / (1 + F_p*G_uw)
H_i = (F_i*G_uw) / (1 + F_i*G_uw)
H_pi = (F_pi*G_uw) / (1 + F_pi*G_uw)

subplot(3,1,1);
step(H_p);
legend("H_p");
subplot(3,1,2);
step(H_i);
legend("H_i");
subplot(3,1,3);
step(H_pi);
legend("H_pi");


%%


K_p_1 = (b*R_a + K_u*K_m)^2 / (4*J*R_a*1*K_m)
K_p_12 = (b*R_a + K_u*K_m)^2 / (4*J*R_a*1/4*K_m)	
K_p_14 = (b*R_a + K_u*K_m)^2 / (4*J*R_a*1/16*K_m)	

subplot(3,1,1);
step((1/(J*R_a) * K_p_1 * K_m ) / ( s^2 + s*1/(J*R_a)*(b * R_a + K_u * K_m) + 1/(J*R_a) * K_p_1 * K_m));
legend("H_p");
subplot(3,1,2);
step((1/(J*R_a) * K_p_12 * K_m ) / ( s^2 + s*1/(J*R_a)*(b * R_a + K_u * K_m) + 1/(J*R_a) * K_p_12 * K_m));
legend("H_i");
subplot(3,1,3);
step((1/(J*R_a) * K_p_14 * K_m ) / ( s^2 + s*1/(J*R_a)*(b * R_a + K_u * K_m) + 1/(J*R_a) * K_p_14 * K_m));
legend("H_pi");







