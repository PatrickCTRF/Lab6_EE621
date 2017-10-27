% Maria Milena         173906  
% Patrick de Carvalho  175480
% Rafael Pereira       176219
% Rafael Nogueira      147704

clc
clear all
close all

% Experimento 6 - Gerador e motor síncrono.

R12 = 3.4;
R23 = 3.4;
R31 = 3.3;
Ra = (R12+R23+R31)/6;

Xd_sat = 23.1153;
Xq_sat = 11.1158;

Xd_nsat = 42.1265;
Xq_nsat = 18.7814;

Vt=127;
Ef=125;

delta = 0:0.01:pi;

P=((Vt * Ef * sin(delta))/ Xd_nsat ) + ((Vt^2) * (Xd_nsat - Xq_nsat) * sin(delta*2))/ (2*Xd_nsat*Xq_nsat);


% figure
% plot(delta, P, 'b.-');
% title(' Curva Potência ');
% ylabel('Potência Trifásica [W]');
% xlabel('Ângulo [rad]');
% grid on;


%------------ LEVANTAMENTO DA CURVA (P - %DELTA)--------------------------

P = [ 377 387 449 536 667 ];
Ia = [0.975 0.997 1.16 1.38 1.71 ];
Vt = [ 224 224 224 225 225 ]; Vt = Vt/sqrt(3);  %% Vt de fase!
Fp = [ 1 1 1 1 1 ];                             
If = [ 0.278 0.272 0.274 0.277 0.277 ];

for i=1:length(Ia)
    angulo_fi(i) = acos(Fp(i));
    angulo_delta(i) = -phase(Vt(i) - Ra*Ia(i) - sqrt(-1)*Xq_sat*Ia(i));
    
    angulo_tridente(i) = angulo_delta(i) + angulo_fi(i);
    
    Id(i) = (Ia(i)*sin(angulo_tridente(i)))*(cos(angulo_delta(i)) + sqrt(-1)*sin(angulo_delta(i)));
    Iq(i) = (Ia(i)*cos(angulo_tridente(i)))*(cos(angulo_delta(i)) + sqrt(-1)*sin(angulo_delta(i)));
    
    Ef(i) = Vt(i) - Ra*Ia(i) - sqrt(-1)*Xq_sat*Iq(i) - sqrt(-1)*Xd_sat*Id(i);
    
% %     P_calculado(i)=((abs(Vt(i)) * abs(Ef(i)) * sin(angulo_delta(i)))/ Xd_sat) + ((abs(Vt(i))^2) * (Xd_sat - Xq_sat) * sin(angulo_delta(i)*2))/ (2*Xd_sat*Xq_sat);
    
    
end

for i=1:length(Ia)
    
    for j=1:length(delta)
        P_calculado(i,j)=((abs(Vt(i)) * abs(Ef(i)) * sin(delta(j)))/ Xd_sat) + ((abs(Vt(i))^2) * (Xd_sat - Xq_sat) * sin(delta(j)*2))/ (2*Xd_sat*Xq_sat);

    end
    
    figure
    plot(delta, 3*P_calculado(i,:),angulo_delta(i),P(i),'xr');
    title(' Curva Potência ');
    ylabel('Potência Trifásica [W]');
    xlabel('Ângulo [rad]');
    grid on;
    
end


%------------ LEVANTAMENTO DA CURVA V sem carga--------------------------------



If_sem_carga = [ 0.006 0.050 0.103 0.153 0.196 0.251 0.303 0.355 0.394 0.453 0.502 0.556 0.602];
Ia_sem_carga = [ 4.21 3.52 2.72 1.95 1.43 0.931 0.463 1.43 1.58 2.08 2.95 3.57 4.33];
Fp = [ 0.2 0.23 0.26 0.36 0.4 0.89 0.97 -0.78 -0.44 -0.23 -0.25 -0.27 -0.3];
% Vt = [ 226 226 226 226 226 227 227 226 226 227 226 227 227];


figure
plot(If_sem_carga, Ia_sem_carga,If_sem_carga, abs(Fp),'xr-');
title(' Curva V em vazio');
ylabel('Corrente Ia e Fp');
xlabel('Corrente de excitação');
grid on;


%------------ LEVANTAMENTO DA CURVA V com carga--------------------------------



If = [ 0.006 0.050 0.103 0.149 0.2 0.25 0.303 0.356 0.4 0.448 0.5 0.55 0.6];
Ia = [ 4.59 3.84 3.02 2.36 1.73 1.33 1.26 1.75 2.11 2.58 3.11 3.77 4.33];
Fp = [ 0.3 0.33 0.42 0.51 0.64 0.9 0.99 0.84 0.68 0.54 0.38 0.38 0.21];


figure
plot(If, Ia,If, abs(Fp),'xr-');
title(' Curva V com carga');
ylabel('Corrente Ia e Fp');
xlabel('Corrente de excitação');
grid on;


figure
plot(If, Ia, If_sem_carga, Ia_sem_carga,'xr-');
title(' Curva V com carga');
ylabel('Corrente Ia com carga e em vazio (vermelho X)');
xlabel('Corrente de excitação');
grid on;


