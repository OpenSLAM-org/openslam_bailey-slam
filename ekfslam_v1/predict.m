function [x,P]= predict (x,P,v,g,Q,WB,dt)
%function [xn,Pn]= predict (x,P,v,g,Q,WB,dt)
%
% Inputs:
%   x, P - SLAM state and covariance
%   v, g - control inputs: velocity and gamma (steer angle)
%   Q - covariance matrix for velocity and gamma
%   WB - vehicle wheelbase
%   dt - timestep
%
% Outputs: 
%   xn, Pn - predicted state and covariance
%
% Tim Bailey 2004.

s= sin(g+x(3)); c= cos(g+x(3));
vts= v*dt*s; vtc= v*dt*c;

% jacobians   
Gv= [1 0 -vts;
     0 1  vtc;
     0 0 1];
Gu= [dt*c -vts;
     dt*s  vtc;
     dt*sin(g)/WB v*dt*cos(g)/WB];
  
% predict covariance
P(1:3,1:3)= Gv*P(1:3,1:3)*Gv' + Gu*Q*Gu';
if size(P,1)>3
    P(1:3,4:end)= Gv*P(1:3,4:end);
    P(4:end,1:3)= P(1:3,4:end)';
end    

% predict state
x(1:3)= [x(1) + vtc; 
         x(2) + vts;
         pi_to_pi(x(3)+ v*dt*sin(g)/WB)];

