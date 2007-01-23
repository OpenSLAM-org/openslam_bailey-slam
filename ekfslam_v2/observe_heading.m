function observe_heading(phi, useheading)
%function observe_heading(phi, useheading)
%
% Perform state update for a given heading measurement, phi,
% with fixed measurement noise: sigmaPhi
global XX PX

if useheading==0, return, end
sigmaPhi= 0.01*pi/180; % radians, heading uncertainty

H= zeros(1,length(XX));
H(3)= 1;
v= pi_to_pi(phi - XX(3));

KF_cholesky_update(v, sigmaPhi^2, H);
