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
fraction_n= [0.004 0.454 5.55 15.1 10];%numerator of Gavr, transfer function
fraction_dn=[1.6e-05 0.002032 0.04732 0.4286 2.133 8.76 18.01 11]; %denominator of Gavr, transfer function
[A,B,C,D] = tf2ss(fraction_n,fraction_dn) % convert fractional transfer function to A, B, C, D space matrix
%% Initiate close loop system with LQR controller
Q=1*[1 0 0 0 0 0 0;
    0 1000 0 0 0 0 0;
    0 0 10 0 0 0 0;
    0 0 0 1 0 0 0;
    0 0 0 0 1 0 0;
    0 0 0 0 0 1 0; 
    0 0 0 0 0 0 1]; 
%Q=1*eye(7);%positive control weighting matrix, size is similar to A matrix
% p = 2;
% Q = p*C'*C
R = 0.0001; %non-negative state weighting matrix
%% Remember TF to SS is not equal, there is a problem do not directly use tf2ss...%%
% convert to numerator, denominator first
% then use tf2ss(num,denom)
% do not use the following two line, never
% [num,den] = tfdata(Gavr,'v');
% [A,B,C,D] = tf2ss(num, den);
%% Continue %%
K=lqr(A,B,Q,R); %See how many value is there, could be useful for designing Simulink system
Af=A-B*K; % Feedback Subtraction node
sys_lqr=ss(Af,B,C,D);%closed loop system with feedback
t=0:0.01:15;
Slqr=step(sys_lqr,t);
pole(sys_lqr)
% A
figure(1)
plot(t,Slqr)
%sim(AVR_LQR_sim.slx)
I_lqr= stepinfo(sys_lqr)
sserror_lqr=abs(1-Slqr(end))
figure(2)
rlocus(sys_lqr);
figure(3)
bode(sys_lqr)
% ylabel('Terminal Voltage (pu)')
% xlabel('Time (sec)')
% legend('AVR LQR')
