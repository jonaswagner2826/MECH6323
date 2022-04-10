%% simplemu.m
%
% This file creates a 5 by 5 matrix M along with several
% uncertainty block structures. This example can be used to
% explore the dependency of mu(M) on the block structure.

% Example data
i=sqrt(-1);
M=[0.10+0.07*i -0.1538+0.1615*i 0-0.56*i  0+42.0*i  4.-1.4*i 
 0-0.2730*i -0.30-0.28*i 2.86+0.546*i -26.0+72.8*i 13.+5.4600*i
 0.10+0.1750*i 0.0769-0.1077*i -0.40+0.2100*i 5.0+3.5000*i 4.5-0.70*i
  0+0.0021*i -0.0038-0.0021*i 0.0060+0.0112*i 0.2000+0.4200*i  0.0600+0.0140*i
 0.0240+0.0280*i 0+0.0269*i -0.0660+0.0420*i 0+0.70*i -0.4210+0.49*i
];

% Sample uncertainty structures
blka = [5 0];
blkb = [3 0;2 0];
blkc = [1 1;1 1;1 1;2 0];
blkd = [1 1;1 1;1 1;1 1;1 1];
blke = [2 2;2 2;1 1];
blkf = [2 2;3 3];
blkg = [5 5];
blkh = [2 3;3 2];
blki = [1 4;4 1];

% Run easymu
[upp,low,pert,dleft,dright] = easymu(M,blka);
[low,upp]