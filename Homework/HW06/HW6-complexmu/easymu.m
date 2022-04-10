% function [upp,low,pert,dleft,dright] = easymu(M,blk)
%
%  upper and lower bounds for mu(M). M is the input
%  constant matrix and BLK is the block structure.
%
%  See also: MUSSV, MUSSVEXTRACT

function [upp,low,pert,dleft,dright] = easymu(M,blk)
if nargin < 2
    disp(['usage: [upp,low,pert,dleft,dright] = easymu(M,blk)']);
    return
end
[bnds,muinfo] = mussv(M,blk);
[VDelta,VSigma,VLmi] = mussvextract(muinfo);
pert = VDelta;
dleft = VSigma.DLeft;
dright = VSigma.DRight;
upp = bnds(1,1);
low = bnds(1,2);
