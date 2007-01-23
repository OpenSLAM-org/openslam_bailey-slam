function [x,P]= update(x,P,z,R,idf, batch)
% function [x,P]= update(x,P,z,R,idf, batch)
%
% Inputs:
%   x, P - SLAM state and covariance
%   z, R - range-bearing measurements and covariances
%   idf - feature index for each z
%   batch - switch to specify whether to process measurements together or sequentially
%
% Outputs:
%   x, P - updated state and covariance

if batch == 1
    [x,P]= batch_update(x,P,z,R,idf);
else
    [x,P]= single_update(x,P,z,R,idf);
end

%
%

function [x,P]= batch_update(x,P,z,R,idf)

lenz= size(z,2);
lenx= length(x);
H= zeros(2*lenz, lenx);
v= zeros(2*lenz, 1);
RR= zeros(2*lenz);

for i=1:lenz
    ii= 2*i + (-1:0);
    [zp,H(ii,:)]= observe_model(x, idf(i));
    
    v(ii)=      [z(1,i)-zp(1);
        pi_to_pi(z(2,i)-zp(2))];
    RR(ii,ii)= R;
end
        
[x,P]= KF_cholesky_update(x,P,v,RR,H);

%
%

function [x,P]= single_update(x,P,z,R,idf)

lenz= size(z,2);
for i=1:lenz
    [zp,H]= observe_model(x, idf(i));
    
    v= [z(1,i)-zp(1);
        pi_to_pi(z(2,i)-zp(2))];
    
    [x,P]= KF_cholesky_update(x,P,v,RR,H);
end        
