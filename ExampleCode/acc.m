%% acc.m
%
% This file does a robustness analysis of an adaptive cruise control.
% A PID controller is designed to follow a fixed distance, r_desired, 
% behind the preceding vehicle. Then a robustness analysis is used
% to analyze the effect of uncertainty.
%

%% Nominal Vehicle Parameters
%     Mass of vehicle is 2065 kg (unloaded)
%     Assume 2 passengers with mass of 80 kg.
m0 = 2065+(2*80);   % nominal mass in (kg)
c0 = 2*0.43*20;     % nominal wind drag at ~20 m/s  (N*s/m)
P0 = tf(1,[m0 c0 0]);

%% PID Design: K(s) = kp + ki/s + (Kd*s)/(tau s + 1);
% Use PID to place a pole at -p and a complex pair with damping, zeta, 
% and natural frequency, wn
p = 1;
zeta = .9; 
wn =  .5;

kp = m0*(2*zeta*wn*p+wn^2);       % proportional gain
ki = m0*wn^2*p;                   % integral gain
kd = -c0+m0*(p+2*zeta*wn);        % derivative gain
tau = .01;                        % differentiator cut-off

K = kp + tf([kd 0],[tau 1]) + tf(ki,[1 0]);

% Verify that the closed loop poles at placed at the specified locations
% Notice that S0 has one fast pole approximately at -100 rad/sec due
% to the low-pass filter time constant tau in the approximate derivative.
% The other pole locations are close to but not exactly at the desired 
% locations due to this low pass filter.
L0 = P0*K;
S0 = feedback(L0,1);
damp(S0)


%% Uncertainty Modeling 
% We'll model uncertainty in the vehicle mass, drag coefficient, and
% actuator dynamics. We'll show how to explictly construct the LFT 
% uncertainty model using SYSIC.  This is a bit labor intensive so
% we'll also show this can easily be done with Matlab's uncertain objects.

% Uncertainty values
Wm = 0.07;       % 7% uncertainty in mass
Wc = 0.3;        % 30% uncertainty in drag
Wa = tf([1 10/3]/3,[0.05 10/3]);  % Actuator = 1+Wa*Da, ||Da||_infty<=1

% Method 1: 
% Uncertain car model is Fu(M,Delta) where Delta=blkiag(del_m,del_c,Da)
% Form M(s) directly using SYSIC
LFTminv = [-Wm 1; -Wm/m0  1/m0];    % 1/m= F_u(LFTminv,del_m)
LFTc = [0 1; c0*Wc  c0];            % c= F_u(LFTc,del_c)
I1 = tf(1,[1 0]);                   % Integrate acceleration
I2 = tf(1,[1 0]);                   % Integrate velocity

systemnames = 'Wa I1 I2 LFTc LFTminv';
inputvar = '[w{3}; u]';
outputvar = '[LFTminv(1); LFTc(1); Wa; I2]';
input_to_Wa = '[u]';
input_to_I1 = '[LFTminv(2)]';
input_to_I2 = '[I1]';
input_to_LFTc = '[w(2); I1]';
input_to_LFTminv = '[w(1); w(3)+u-LFTc(2)]';
sysoutname = 'M';
cleanupsysic = 'yes';
sysic;

% Method 2: 
% Use Matlab's uncertain objects to form uncertain car model.  Behind
% the scenes this is constructing an LFT model similar to Method 1.
% P1 is the transfer function from force to velocity and I is the
% integrator from velocity to position.  I split these apart so that
% I could easily set the initial condition in the Simulink model
% constructed later in this file.
%m = ureal('m',m0,'Percentage',Wm);
%c = ureal('c',c0,'Percentage',Wc);
m = ureal('m',m0,'Percentage',100*Wm);
c = ureal('c',c0,'Percentage',100*Wc);
Da = ultidyn('Da',[1 1]);
P = ss([0 1; 0 -c/m],[0; 1/m],[1 0],0)*(1+Wa*Da);

% Comparison
% We can do a simple check to confirm that the two methods yield the
% same uncertainty set.  We can draw a random sample from the set
% described by Method 2 using the USAMPLE command.  This function
% will return a plant in the set Ps2 and the values (m,c,Da) used
% to construct Ps2.  We can then convert (m,c) to their normalized
% values (del_m,del_c). These can be used to construct a plant Ps1
% in the uncertainty set described by Method 1 using the LFT
% interconnection of M and the uncertainty.   These two plants are
% identical (within numerical error) as expected.
[Ps2,D] = usample(P,1);
del_m = (D.m-m0)/(Wm*m0);
del_c = (D.c-c0)/(Wc*c0);
D1 = blkdiag(del_m,del_c,D.Da);
Ps1 = lft(D1,M);

figure(1)
bode(Ps1,'b',Ps2,'r--');
legend('Method 1','Method 2');
grid on;

% Note you can get the LFT structure underlying the model construted with
% Method 2 using the LFTDATA function. The outputs are the known part
% M2(s) and the uncertainty block D2(s). M2 won't necessarily be the
% same as the M constructed with Method 1 because the uncertainties
% are in a different order.  For example, we ordered the uncertainties
% as delta_m, delta_c, and Da(s) in Method 1.  However, the model
% constructed in method 2 has the uncertainties in D2 ordered as Da, 
% delta_c and delta_m. In addition, there is some non-uniqueness in
% the representation of the known part (this non-uniqueness is 
% exploited as D-scales in computing robustness upper bounds).
% We won't use this for now but it is useful to know that you can get
% LFT model if needed. We'll use the model P developed using Method 2
% in the remainder of the analysis.
[M2,D2] = lftdata(P);


%% Monte Carlo Analysis
% One simple use of these uncertainty models is to run many simulations
% using random samples drawn from the uncertainty set.  This will give
% a sense of the closed-loop performance for the various plant models in
% the set.  The code below uses an uncertain Simulink block to perform
% the Monte Carlo simulations.   The preceding vehicle was modeled using
% velocity data recorded on an experimental ACC while traveling near
% Palo Alto, California. 

% Initialize simulation model
TFINAL = 25;
r_desired = 30;       % desired following distance (m)
load reftrajdata;     % velocity trajectory from experimental data
v0 = reftraj.vp(1);
Delta_ic = zeros(1,1);

% Simulate nominal response
V = [];
sim('accsim',[0 TFINAL]);	
figure(2);
plot(t,e,'b','LineWidth',3);
xlabel('Time (sec)');
ylabel('Spacing Error, e (m)');
hold on;

figure(3);
plot(t,u/m0,'b','LineWidth',3);
xlabel('Time (sec)');
ylabel('Normalized Control Cmd, u/m0 (m/s^2)');
hold on;

% Simulate closed-loop with several plants drawn from the uncertainty set
Ns = 10;
for i=1:Ns
    % Draw random sample of uncertainty values    
    % V is used in the uncertain state-space Simulink block
    V = usample(ufind('accsim'));
    
    % Simulate system
    sim('accsim',[0 TFINAL]);	
    
    % Plot results
    figure(2);
    plot(t,e,'r-.');    
    figure(3);
    plot(t,u/m0,'r-.');    
end

figure(2)
legend('Nominal','Samples','Location','Best');
grid on;
hold off;

figure(3);
grid on;
legend('Nominal','Samples','Location','Best');
hold off;

%% Robust Stability Analysis

% Form loop, sensitivity, and complementary sensitivity functions
L = P*K;
T = feedback(L,1);
S = feedback(1,L);

% Robust stability analysis
opt=robOptions('Mussv','ag3','Display','on','VaryFrequency','on');
[STABMARG,DESTABUNC,INFO] = robstab(S,opt);
stabmargLB = INFO.Bounds(:,1);
stabmargUB = INFO.Bounds(:,2);

muLB = 1./stabmargUB;
muUB = 1./stabmargLB;


% Plot mu bounds
% Note that the upper and lower bounds are so close that they are 
% basically indistinguishable. Thus we have almost exactly computed
% the structured singular value mu.  The peak on the mu plot is inversely
% related to the stability margin, i.e. stability margin = 1/mu. Thus
% the bounds in STABMARG and the destabilizing frequency are related
% to the bounds and peak on a mu plot.
figure(4)
semilogx(INFO.Frequency,muUB,'b',INFO.Frequency,muLB,'r');
xlim([1e-2 1e2]);
xlabel('Frequency, rad/sec');
ylabel('mu');
grid on;

% Simulate closed-loop system with destabilizing perturbation
% Notice that the closed-loop begins to oscillate at the frequency
% specified by STABMARG.DestabilizingFrequency.
V = DESTABUNC;
sim('accsim',[0 TFINAL]);	
figure(5);
plot(t,e,'b','LineWidth',3);
xlabel('Time (sec)');
ylabel('Tracking error, e (m)');
grid on;

%% Worst-case gain analysis

% Robust stability analysis
[MaxGain,MaxGainUnc,INFO] = wcgain(S);
    
% Plot nominal and worst-case sensitivity
Swc = usubs(S,MaxGainUnc);
[norm(S.Nominal,inf) norm(Swc,inf,1e-3) MaxGain.LowerBound]

figure(6)
bodemag(S.Nominal,'b',Swc,'r',{1e-1,1e1});
ylim([-40 10]);

% Simulate closed-loop system with worst-case gain perturbation
V = MaxGainUnc;
sim('accsim',[0 TFINAL]);	
figure(7);
plot(t,e,'b','LineWidth',3);
xlabel('Time (sec)');
ylabel('Tracking error, e (m)');
grid on;
