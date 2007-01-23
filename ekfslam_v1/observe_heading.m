function [x,P]= observe_heading(x,P, phi, useheading)
%function [x,P]= observe_heading(x,P, phi, useheading)
%
% Perform state update for a given heading measurement, phi,
% with fixed measurement noise: sigmaPhi

if useheading==0, return, end
sigmaPhi= 0.01*pi/180; % radians, heading uncertainty

H= zeros(1,length(x));
H(3)= 1;
v= pi_to_pi(phi - x(3));

[x,P]= KF_joseph_update(x,P,v, sigmaPhi^2,H);
