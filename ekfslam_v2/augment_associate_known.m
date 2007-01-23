function table= augment_associate_known(z,R,idz, table)
%%% OBSOLETE FUNCTION 
error('Obsolete function')

%function table = augment_associate_known(z,R,idz, table)
%
% Add the new feature indices to the data-association lookup table
% and then add the features to the state. 
global XX

Nxv= 3; % number of vehicle pose states
Nf= (length(XX) - Nxv)/2; % number of features already in map

table(idz)= Nf + (1:size(z,2)); % add new feature positions to lookup table
augment(z,R); % add new features to state
