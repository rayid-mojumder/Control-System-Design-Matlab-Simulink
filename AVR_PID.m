%% Define the gain parameters for Amplifier (Ka), Exciter (Ke), Generator (Kg), and Sensor (Ks)
Ka=10;
Ke=1;
Kg=1;
Ks=1;
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
Gavr=Gnum/(1+Gnum*Gdenum);
%Initiate open loop system without controller
t=0:0.01:15;
S=step(Gavr,t);
%I= stepinfo(Gavr) %get rise time, settling time, peak overshoot, etc.
hold on
% Analysis plot
pole(Gavr);
% figure(1);
% rlocus(Gavr);
%% Initiate close loop system with PID controller
Kp=0.70958;
Ki=0.54015;
Kd=0.19211;
Gpid=Kp+Ki*(1/s)+Kd*s;
Gnum1=Gpid*Ga*Ge*Gg;
Gavrpid=Gnum1/(1+Gnum1*Gdenum);
t=0:0.01:15;
Spid=step(Gavrpid,t)
pole(Gavrpid);
%plot(t,Spid)
%I_pid= stepinfo(Gavrpid)
%sserror_pid=abs(1-Spid(end))
%rlocus(Gavrpid);
bode(Gavrpid)