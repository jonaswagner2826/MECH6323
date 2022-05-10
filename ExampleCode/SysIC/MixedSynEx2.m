% 04/12/2020
%
% Mixed Synthesis Example 2cod
% 
% This example demonstrates the use of sysic/connect on a simple
% mixed synthesis example with a 2-DOF controller. The mixsyn command
% does not support this more general architecture. Instead the
% generalized plant must be constructed with sysic or connect.

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
% This is a **single DOF** design for comparison
[K,CL,GAM,INFO]=mixsyn(P,WS,WK,WT);

%% Solve using HINFSYN
% The mixed sensitivity design is a special case of the more general
% H-infinity problem.  This section uses SYSIC and CONNECT to build
% the generalized plant to solve the MIXSYN problem.  The results from
% HINFSYN with is generalized plant should match the results from MIXSYN.

% SYSIC Syntax
systemnames = 'P WS WK';
inputvar = '[r; u]';
outputvar = '[WS; WK; r; P]';
input_to_P = '[u]';
input_to_WS = '[r-P]';
input_to_WK = '[u]';
Gs = sysic;  

% Call HINFSYN on generalized plant from SYSIC
ny=2;
nu=1;
[Ks,CLs,GAMs,INFOs]=hinfsyn(Gs,ny,nu);


% CONNECT Syntax
P.u = 'u'; P.y = 'y';
WS.u = 'e'; WS.y = 'etil';
WK.u = 'u'; WK.y = 'util';
Sum = sumblk('e = r - y');
Gc = connect(P,WS,WK,Sum,{'r','u'},{'etil','util','r', 'y'});

% Call HINFSYN on generalized plant from CONNECT
ny=2;
nu=1;
[Kc,CLc,GAMc,INFOc]=hinfsyn(Gc,ny,nu);

%% Compare Results

% There is no benefit to the 2DOF design Based on the final "GAM" values
[GAM GAMs GAMc]

% Create closed-loop (without weights) using SYSIC 
systemnames = 'P Ks';
inputvar = 'r';
outputvar = '[r-P; Ks]';
input_to_P = '[Ks]';
input_to_Ks = '[r; P]';
Gunwt = sysic;  

figure;
subplot(1,2,1)
title('K(r to u)');
bode(K,'b',Ks(1),'r--',Kc(1),'g-.');
subplot(1,2,2)
title('K(y to u)');
bode(-K,'b',Ks(2),'r--',Kc(2),'g-.');

figure;
S = feedback(1,P*K);
bodemag(S,'b',Gunwt(1,1),'r--',BS,'c');

figure;
KS = feedback(K,P);
bodemag(KS,'b',Gunwt(2,1),'r--',BK,'c');
