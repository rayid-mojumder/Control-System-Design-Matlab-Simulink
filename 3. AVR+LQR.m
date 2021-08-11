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
% convert fractional transfer function to A, B, C, D space matrix
[num,den] = tfdata(Gavr,'v');
[A,B,C,D]=tf2ss(num, den)
[K, P, E]=lqr(A,B,Q,R);
K
Af=A-B*K; % Feedback Subtraction node
sys1=ss(Af,B,C,D);%closed loop system with feedback]
t=0:0.01:15;
Slqr=step(sys1,t);
plot(t,S,'b',t,Spid,'r',t,Slqr,'g')
ylabel('Terminal Voltage (pu)')
xlabel('Time (sec)')
legend('AVR','AVR PID','AVR LQR')