function update(z,R,idf, batch)
% function update(z,R,idf, batch)
%
% Inputs:
%   z, R - range-bearing measurements and covariances
%   idf - feature index for each z
%   batch - switch to specify whether to process measurements together or sequentially
%
% Outputs:
%   XX, PX - updated state and covariance (global variables)

if batch == 1
    batch_update(z,R,idf);
else
    single_update(z,R,idf);
end

%
%

function batch_update(z,R,idf)
global XX PX

lenz= size(z,2);
lenx= length(XX);
H= zeros(2*lenz, lenx);
v= zeros(2*lenz, 1);
RR= zeros(2*lenz);

for i=1:lenz
    ii= 2*i + (-1:0);
    [zp,H(ii,:)]= observe_model(XX, idf(i));
    
    v(ii)=      [z(1,i)-zp(1);
        pi_to_pi(z(2,i)-zp(2))];
    RR(ii,ii)= R;
end
        
KF_cholesky_update(v,RR,H);

%
%

function single_update(z,R,idf)
global XX PX

lenz= size(z,2);
for i=1:lenz
    [zp,H]= observe_model(XX, idf(i));
    
    v= [z(1,i)-zp(1);
        pi_to_pi(z(2,i)-zp(2))];
    
    KF_cholesky_update(v,RR,H);
end        
