% 04/12/2020
%
% Mixed Synthesis Example 1
% 
% This example demonstrates the use of sysic/connect on a simple
% mixed synthesis example.

clear
format short e
format compact
close all
clc

%% Problem Data

% Plant
P=ss(-1,2,3,4);   

% Sensitivity bound / weight
wS=10;     
AS=1/1000;  
MS=2;   
BS = tf([MS AS*wS],[1 wS]);
WS = 1/BS;

% Control effort bound / weight
umax = 3;
r0 = 10;
wK = 5*wS;
AK = umax/r0;
MK = AK/100;
BK = tf([MK AK*wK],[1 wK]);
WK = 1/BK;

% Complementary Sensitivity bound / weight (neglected);
WT = [];     

%% Mixed Synthesis Design

[K,CL,GAM,INFO]=mixsyn(P,WS,WK,WT);

%% Solve using HINFSYN
% The mixed sensitivity design is a special case of the more general
% H-infinity problem.  This section uses SYSIC and CONNECT to build
% the generalized plant to solve the MIXSYN problem.  The results from
% HINFSYN with is generalized plant should match the results from MIXSYN.

% SYSIC Syntax
systemnames = 'P WS WK';
inputvar = '[r; u]';
outputvar = '[WS; WK; r-P]';
input_to_P = '[u]';
input_to_WS = '[r-P]';
input_to_WK = '[u]';
Gs = sysic;  

% Call HINFSYN on generalized plant from SYSIC
ny=1;
nu=1;
[Ks,CLs,GAMs,INFOs]=hinfsyn(Gs,ny,nu);

% CONNECT Syntax
P.u = 'u'; P.y = 'y';
WS.u = 'e'; WS.y = 'etil';
WK.u = 'u'; WK.y = 'util';
Sum = sumblk('e = r - y');
Gc = connect(P,WS,WK,Sum,{'r','u'},{'etil','util','e'});

% Call HINFSYN on generalized plant from CONNECT
ny=1;
nu=1;
[Kc,CLc,GAMc,INFOc]=hinfsyn(Gc,ny,nu);

%% Compare Results

[GAM GAMs GAMc]

figure;
bode(K,'b',Ks,'r--',Kc,'g-.');

figure;
S = feedback(1,P*K);
Ss = feedback(1,P*Ks);
Sc = feedback(1,P*Kc);
bodemag(S,'b',Ss,'r--',Sc,'g-.',BS,'c');

figure;
KS = feedback(K,P);
KSs = feedback(Ks,P);
KSc = feedback(Kc,P);
bodemag(KS,'b',KSs,'r--',KSc,'g-.',BK,'c');
