function KF_simple_update(v,R,H)
%function KF_simple_update(v,R,H)
%
% Calculate the KF (or EKF) update given the prior state [x,P]
% the innovation [v,R] and the (linearised) observation model H.
% The result is calculated using a naive inversion of S, and is
% less numerically stable than the Cholesky factorisation based 
% update (see KF_cholesky_update).
%
% Tim Bailey 2003
global XX PX

PHt= PX*H';
S= H*PHt + R;
Si= inv(S);
Si= make_symmetric(Si); % ensure is symmetric
PSD_check= chol(Si);
W= PHt*Si;

XX= XX + W*v; 
PX= PX - make_symmetric(W*S*W');
PSD_check= chol(PX);

function P= make_symmetric(P)
P= (P+P')*0.5;
