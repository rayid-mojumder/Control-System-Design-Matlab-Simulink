%% Define the gain parameters for Amplifier (Ka), Exciter (Ke), Generator (Kg), and Sensor (Ks)
Ka=10;
Ke=1.0;
Kg=1.0;
Ks=1.0;
%% Define the time constant for Amplifier (Ta), Exciter (Te), Generator (Tg), and Sensor (Ts)
Ta=0.1;
Te=0.4;
Tg=1;
Ts=0.01;
%% Define transfer function for Amplifier (Ga), Exciter (Ge), Generator (Gg), and Sensor (Gs)
s = tf('s');
Ga=Ka/(1+Ta*s);
Ge=Ke/(1+Te*s);
Gg=Kg/(1+Tg*s);
Gs=Ks/(1+Ts*s);
%% Create close loop transfer function for the AVR system
Gnum=Ga*Ge*Gg;
Gdenum=Gs;
Gavr=Gnum/(1+Gnum*Gdenum)
%Initiate open loop system without controller
t=0:0.01:15;
[S,t1]=step(Gavr,t);
I= stepinfo(Gavr);
sserror=abs(1-S(end));%get steady-state error
hold on
% Analysis plot
pole(Gavr);
%plot(t,S);
% figure
%rlocus(Gavr);
%bode(Gavr),grid;
