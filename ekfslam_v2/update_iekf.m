function update_iekf(z,R,idf,N)
% function update_iekf(z,R,idf,N)
%
% Inputs:
%   z, R - range-bearing measurements and covariances
%   idf - feature index for each z
%   N - number of iterations of the IEKF
%
% Outputs:
%   XX, PX - updated state and covariance (globals)

global XX PX IDF
IDF= idf;

if isempty(idf), return, end

lenz= size(z,2);
RR= zeros(2*lenz);
zz= zeros(2*lenz,1);
for i=1:lenz
    ii= 2*i + (-1:0);    
    zz(ii)= z(:,i);
    RR(ii,ii)= R;
end
        
[XX,PX] = KF_IEKF_update(XX,PX, zz,RR, @hmodel, @hjacobian, N);

%
%

function v= hmodel(x,z)
global IDF
lenz= length(IDF);
v= zeros(2*lenz, 1);

for i=1:lenz
    ii= 2*i + (-1:0);    
    [zp,dmy]= observe_model(x, IDF(i));
    v(ii)= z(ii)-zp;
    v(ii(2))= pi_to_pi(v(ii(2)));
end

%
%

function H= hjacobian(x)
global IDF

lenz= length(IDF);
lenx= length(x);
H= zeros(2*lenz, lenx);

for i=1:lenz
    ii= 2*i + (-1:0);
    [dmy,H(ii,:)]= observe_model(x, IDF(i));
end
