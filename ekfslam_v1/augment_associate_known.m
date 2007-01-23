function [x,P,table]= augment_associate_known(x,P, z,R,idz, table)
%function [x,P,table]= augment_associate_known(x,P, z,R,idz, table)
%
% Add the new feature indices to the data-association lookup table
% and then add the features to the state. 

Nxv= 3; % number of vehicle pose states
Nf= (length(x) - Nxv)/2; % number of features already in map

table(idz)= Nf + (1:size(z,2)); % add new feature positions to lookup table
[x,P]= augment(x,P,z,R); % add new features to state
