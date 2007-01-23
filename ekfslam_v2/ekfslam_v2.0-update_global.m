function update_global(z,R,idf, batch)
% function update_global(z,R,idf, batch)
%
% Inputs:
%   z, R - range-bearing measurements and covariances
%   idf - feature index for each z
%   batch - switch to specify whether to process measurements together or sequentially
%
% Outputs:
%   XX, PX - updated state and covariance (global variables)
%
% Tim Bailey 2004. Based on observation model derived by Jose Guivant.

global XX PX

lenx= size(XX,1);
lenz= size(z,2);

% storage
H= zeros(2*lenz, lenx);
v= zeros(2*lenz, 1);
RR= zeros(2*lenz);

% transform observations
for i=1:lenz
    ii= 2*i + (-1:0);
    jj= idf(i)*2 + (2:3);

    c = cos(XX(3) + z(2,i));
    s = sin(XX(3) + z(2,i));    
    rc = z(1,i) * c;
    rs = z(1,i) * s;
    
    v(ii) = XX(jj) - XX(1:2) - [rc; rs];
    v(ii) = -v(ii);
    
    H(ii,1:3) = [-1 0 rs; 0 -1 -rc];
    H(ii,jj) = [1 0; 0 1];
    
    HR = [-c rs; -s -rc];
    RR(ii,ii) = HR * R * HR';
end

KF_cholesky_update(v,RR,H);
