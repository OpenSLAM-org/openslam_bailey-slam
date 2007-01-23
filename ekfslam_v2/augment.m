function augment(z,R)
%function augment(z,R)
%
% Inputs:
%   z, R - range-bearing measurements and covariances, each of a new feature
%
% Outputs:
%   x, P - augmented SLAM state and covariance (global variables)
%
% Notes: 
%   - We assume the number of vehicle pose states is three.
%   - Only one value for R is used, as all measurements are assumed to 
%   have same noise properties.
%
% Tim Bailey 2004.

% add new features to state
for i=1:size(z,2)
    add_one_z(z(:,i),R);
end

%
%

function add_one_z(z,R)
global XX PX

len= length(XX);
r= z(1); b= z(2);
s= sin(XX(3)+b); 
c= cos(XX(3)+b);

% augment x
XX= [XX;
     XX(1) + r*c;
     XX(2) + r*s];

% jacobians
Gv= [1 0 -r*s;
     0 1  r*c];
Gz= [c -r*s;
     s  r*c];
     
% augment P
rng= len+1:len+2;
PX(rng,rng)= Gv*PX(1:3,1:3)*Gv' + Gz*R*Gz'; % feature cov
PX(rng,1:3)= Gv*PX(1:3,1:3); % vehicle to feature xcorr
PX(1:3,rng)= PX(rng,1:3)';
if len>3
    rnm= 4:len;
    PX(rng,rnm)= Gv*PX(1:3,rnm); % map to feature xcorr
    PX(rnm,rng)= PX(rng,rnm)';
end
