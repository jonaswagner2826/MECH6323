% 04/12/2020
%
% Mixed Synthesis Example 3
% 
% This example demonstrates the use of sysic/connect to build a simple
% model matching interconnection. The data is not meant to represent
% a real problem but simply to show how to build the interconnection.

clear
format short e
format compact
close all
clc

%% Problem Data

% Plant
P=ss(-1,2,3,4);   
Act = tf(10,[1 10]);

% Weights
Wn = ss(1);
Wp = ss(2);
Model = tf(9,[1 2*0.7*3 9]);
Wu = ss(3);
Wd = ss(4);
Wcmd = ss(5);

%% Interconnection with SYSIC

systemnames = 'P Act Model Wn Wd Wp Wu Wcmd';
inputvar = '[r; d; n; u]';
outputvar = '[Wp; Wu; Wcmd; P+Wn]';
input_to_P = '[Act]';
input_to_Act = '[u+Wd]';
input_to_Model = '[Wcmd]';
input_to_Wn = '[n]';
input_to_Wd = '[d]';
input_to_Wp = '[Model-P]';
input_to_Wu = '[u]';
input_to_Wcmd = '[r]';
Gs = sysic;

%% Interconnection with CONNECT

P.u = 'uP'; P.y = 'y';
Act.u = 'uAct'; Act.y = 'uP';
Model.u = 'r'; Model.y = 'ydes';
Wn.u = 'ntil'; Wn.y = 'n';
Wd.u = 'dtil'; Wd.y = 'd';
Wp.u = 'e'; Wp.y = 'etil';
Wu.u = 'u'; Wu.y = 'util';
Wcmd.u = 'rtil'; Wcmd.y = 'r';
eSum = sumblk('e = ydes - y');
mSum = sumblk('m = y + n');
dSum = sumblk('uAct = u+d');
Gc = connect(P,Act,Model,Wn,Wd,Wp,Wu,Wcmd,eSum,mSum,dSum,...
    {'rtil','dtil','ntil','u'},{'etil','util','r', 'm'});

%% Compare Results

norm(Gs-Gc,inf)

