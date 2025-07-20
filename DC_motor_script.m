clear
clc
close all
%% MATLAB CODE

%excitation circuit parameters
Ve_r=60; %excitation rated voltage
Ie_r=5; %excitation rated current
Re=12; %excitation resistance
tau_e=0.1; %excitation time constant [s]
Le=tau_e*Re; 


%other parameters
mt=15000; %mass of the vehicle at no load [Kg]
m_max=mt+130*80; %mass of the vehicle+added passengers at maximum loading capacity


ia_r=156; %overall rated current [A]
Pr_tot=21000*4; %total rated power for the equivalent DC motor [W]
Va_r=600; %line voltage=rated armature voltage (upper limit in the saturation block)

d= 680e-3; %diameter of the wheel [m]
rho=13/74; %gearbox ratio (motor-to-wheels)

v_max=42; %maximum speed of the vehicle [km/h]
omega_max=(v_max/3.6)*2/(d*rho); %maximum "structural" mechanical limit [rad/s] 
omega_b=970*(2*pi/60); %[rad/s] rated angular speed of the motor (the base speed)-->at this speed we are applying the rated armature voltage
v_r=omega_b*rho*(d/2)*3.6; %rated linear speed


%machine parameters (ARMATURE PART)
Ra = 0.39;
tau_a = 10e-3; %time constant of the electrical part [s]
La = tau_a*Ra;
J = m_max*((rho^2*d^2)/4); %equivalent inertia seen by the motor, calculated in the WORST CASE SCENARIO (full loading capacity)
B = 0.81; %"beta"-->friction coefficient
Ks = 1.06; %called "Kt" in the dataset-->under some assumption the torque is considered equal to the back emf constant 
Tn= Pr_tot/omega_b;
E_n=Ks*Ie_r*omega_b;

%% definition of the system TF

s = tf('s');

Gi = 1/(Ra+La*s); %electrical part
GO = 1/(B+J*s); %mechanical part

Ge = 1/(Re+Le*s); %excitation part


%imposition of the critical pulsations
wc_a=500; 
wc_m=5;

wc_e=50; 

%at the end the system must be stable, so we also request am high phase margin (>=80°, avoiding oscillating dynamics)
phase_m_curr = 90;
phase_m_speed = 90;

phase_m_exc=90;


%% PID TUNER approach, to obtain the expression of the desired PI controller (imposing also the phase margin)
%re-writing the phase margin to insert it in the pidtune function
opt_curr = pidtuneOptions('PhaseMargin',phase_m_curr);
opt_speed = pidtuneOptions('PhaseMargin',phase_m_speed);

%CONTROLLER FOR EXCITATION PART (like the one of the electrical part, using Le and Re)
opt_exc_curr = pidtuneOptions('PhaseMargin',phase_m_exc);


[Ccurr, info_curr] = pidtune(Gi,'pi',wc_a,opt_curr); %we request a goal dynamic of wc_a and a phase margin opt_curr
[Cspeed, info_speed] = pidtune(GO,'pi',wc_m,opt_speed);

[Cexc_curr, info_exc_curr] = pidtune(Ge,'pi',wc_e,opt_exc_curr);

%we build up the obtained controllers
R_curr = Ccurr.Kp+Ccurr.Ki/s; 
R_speed = Cspeed.Kp+Cspeed.Ki/s;

R_exc_curr = Cexc_curr.Kp+Cexc_curr.Ki/s;

%open loop TF
L_curr = R_curr*Gi;
L_speed = R_speed*GO;

L_exc_curr = R_exc_curr*Ge;

%closed loop TF (built in an independent way)
F_curr = feedback(L_curr,1);
F_speed = feedback(L_speed,1);

F_exc_curr = feedback(L_exc_curr,1);

%NESTING the speed controller and the speed TF around the closed loop TF of the current
%-->this is due to the COMPENSATION OF THE EMF ("E_est"), which generate an equivalent decoupled scheme
F_comp = feedback(R_speed*F_curr*GO,1);



%% definition of slope/speed look-up tables
S=[0;0;0.05;0;0;-0.05;0;0]; %slope
L=[0;1;3;4;6;8;9;10]; %time intervals
V=[v_r/2;v_r;v_r;v_max;v_r;v_r;v_r/2;0]; %speed



%% plots and analysis

% Bode plot of the current loop (inner loop)-->behave similar to a pure integrator
opt = bodeoptions;

opt.XLabel.Interpreter = 'latex';
opt.YLabel.Interpreter = 'latex';
opt.Title.Interpreter = 'latex';

opt.XLabel.FontSize = 12;
opt.YLabel.FontSize = 12;
opt.Title.FontSize = 15;
%% armature current plot
figure
bode(L_curr,'b',opt)
hold on
bode(F_curr,'r',opt)
grid on
%MODIFIED CODE WITH CHAT_GPT
axBode = findall(gcf, 'Type', 'axes'); % Trova tutti gli assi della figura
for i = 1:length(axBode)
    axBode(i).TickLabelInterpreter = 'latex';
end

leg = legend({'$\left| L(s) \right|$', '$\left| F(s) \right|$'});
set(leg, 'Interpreter', 'latex', 'FontSize', 12, 'Location', 'southeast');
title('Current Loop')


%% Bode plot of the speed loop (outer loop)--> considered "alone", without the other controller-->behave similar to a pure integrator

figure
bode(L_speed,'b',opt)
hold on
bode(F_speed,'r',opt)
grid on
%MODIFIED CODE WITH CHAT_GPT
axBode = findall(gcf, 'Type', 'axes'); % Trova tutti gli assi della figura
for i = 1:length(axBode)
    axBode(i).TickLabelInterpreter = 'latex';
end

leg = legend({'$\left| L(s) \right|$', '$\left| F(s) \right|$'});
set(leg, 'Interpreter', 'latex', 'FontSize', 12, 'Location', 'southeast');
title('Speed Loop')

%% excitation current plot
figure
bode(L_exc_curr,'b',opt)
hold on
bode(F_exc_curr,'r',opt)
grid on
%MODIFIED CODE WITH CHAT_GPT
axBode = findall(gcf, 'Type', 'axes'); % Trova tutti gli assi della figura
for i = 1:length(axBode)
    axBode(i).TickLabelInterpreter = 'latex';
end

leg = legend({'$\left| L(s) \right|$', '$\left| F(s) \right|$'});
set(leg, 'Interpreter', 'latex', 'FontSize', 12, 'Location', 'southeast');
title('Excitation Current Loop')



%we plot the closed loop TF of our cascade-->if our decoupling worked fine, we should espect the same bode plot as the Speed Loop has taken alone, because we are considering a very large separation in frequency btw the speed loop and the current loop 
%-->as expecting, they are correctly overlapping!--> decoupling strategy went well!

figure
bode(F_comp,'b',opt)
grid on
%MODIFIED CODE WITH CHAT_GPT
axBode = findall(gcf, 'Type', 'axes'); % Trova tutti gli assi della figura
for i = 1:length(axBode)
    axBode(i).TickLabelInterpreter = 'latex';
end

leg = legend({'$\left| F_comp(s) \right|$'});
set(leg, 'Interpreter', 'latex', 'FontSize', 12, 'Location', 'southeast');
title('Complete Control Scheme')


%%  STEP RESPONSES

[y1,t1] = step(F_curr);
[y2,t2] = step(F_speed);
[y3,t3] = step(F_comp);

[y4,t4] = step(F_exc_curr);

%1st figure-->quite fast dynamic of the control loop (at steady state in 100ms)
figure
fig1 = plot(t1,y1,'b');
grid on

xlabel('Time $\left[\mathrm{s}\right]$','interpreter','latex', 'FontSize', 30)
ylabel('Current $\left[\mathrm{A}\right]$','interpreter','latex', 'FontSize', 30)
set(gca, 'FontSize', 30, 'LineWidth', 1,'FontName','times new roman')
set(fig1,'LineWidth',3)
set(gca,'TickLabelInterpreter', 'latex');
title('Current control')
%xlim([0,2])

%2nd figure
figure
fig1 = plot(t2,y2,'b');
grid on

xlabel('Time $\left[\mathrm{s}\right]$','interpreter','latex', 'FontSize', 30)
ylabel('Speed $\left[\mathrm{rad/s}\right]$','interpreter','latex', 'FontSize', 30)
set(gca, 'FontSize', 30, 'LineWidth', 1,'FontName','times new roman')
set(fig1,'LineWidth',3)
set(gca,'TickLabelInterpreter', 'latex');
title('Speed control')
%xlim([0,2])

%3rd figure
figure
fig1 = plot(t3,y3,'b');
grid on

xlabel('Time $\left[\mathrm{s}\right]$','interpreter','latex', 'FontSize', 30)
ylabel('Speed $\left[\mathrm{rad/s}\right]$','interpreter','latex', 'FontSize', 30)
set(gca, 'FontSize', 30, 'LineWidth', 1,'FontName','times new roman')
set(fig1,'LineWidth',3)
set(gca,'TickLabelInterpreter', 'latex');
title('Complete scheme')
%xlim([0,2])

%4th figure-->excitation current 
figure
fig1 = plot(t4,y4,'b');
grid on

xlabel('Time $\left[\mathrm{s}\right]$','interpreter','latex', 'FontSize', 30)
ylabel('Current $\left[\mathrm{A}\right]$','interpreter','latex', 'FontSize', 30)
set(gca, 'FontSize', 30, 'LineWidth', 1,'FontName','times new roman')
set(fig1,'LineWidth',3)
set(gca,'TickLabelInterpreter', 'latex');
title('Excitation Current control')


%% compute pole/zeros/gains explicitly to check results

[zi,pi,ki] = tf2zp(cell2mat(F_curr.Numerator),cell2mat(F_curr.Denominator));
[zo,po,ko] = tf2zp(cell2mat(F_speed.Numerator),cell2mat(F_speed.Denominator));
[zc,pc,kc] = tf2zp(cell2mat(F_comp.Numerator),cell2mat(F_comp.Denominator)); %if we put together 2 systems which are decoupled from a dynamical point of view, we end up with a dynamic that is the "sum" of the 2 (4 poles are the sum of the 2 couple of poles)

[ze,pe,ke] = tf2zp(cell2mat(F_exc_curr.Numerator),cell2mat(F_exc_curr.Denominator));
% visualize poles/zeros --> we have all NEGATIVE and REAL poles, so we have all STABLE poles!

figure
zplane(cell2mat(F_curr.Numerator),cell2mat(F_curr.Denominator))
title('Current control')
grid on

figure
zplane(cell2mat(F_speed.Numerator),cell2mat(F_speed.Denominator))
title('Speed control')
grid on

figure
zplane(cell2mat(F_comp.Numerator),cell2mat(F_comp.Denominator))
title('Complete scheme')
grid on

figure
zplane(cell2mat(F_exc_curr.Numerator),cell2mat(F_exc_curr.Denominator))
title('Excitation Current control')
grid on