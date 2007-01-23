function data= ekfslam_sim(lm, wp)
%function data= ekfslam_sim(lm, wp)
%
% INPUTS: 
%   lm - set of landmarks
%   wp - set of waypoints
%
% OUTPUTS:
%   data - a data structure containing:
%       data.true: the vehicle 'true'-path (ie, where the vehicle *actually* went)
%       data.path: the vehicle path estimate (ie, where SLAM estimates the vehicle went)
%       data.state(k).x: the SLAM state vector at time k
%       data.state(k).P: the diagonals of the SLAM covariance matrix at time k
%
% NOTES:
%   This program is a SLAM simulator. To use, create a set of landmarks and 
%   vehicle waypoints (ie, waypoints for the desired vehicle path). The program
%   'frontend.m' may be used to create this simulated environment - type
%   'help frontend' for more information.
%       The configuration of the simulator is managed by the script file
%   'configfile.m'. To alter the parameters of the vehicle, sensors, etc
%   adjust this file. There are also several switches that control certain
%   filter options.
%
% Tim Bailey and Juan Nieto 2004.
% Version 1.0


format compact
configfile; % ** USE THIS FILE TO CONFIGURE THE EKF-SLAM **

% setup plots
fig=figure;
plot(lm(1,:),lm(2,:),'b*')
hold on, axis equal
plot(wp(1,:),wp(2,:), 'g', wp(1,:),wp(2,:),'g.')
xlabel('metres'), ylabel('metres')
set(fig, 'name', 'EKF-SLAM Simulator')
h= setup_animations;
veh= [0 -WHEELBASE -WHEELBASE; 0 -2 2]; % vehicle animation
plines=[]; % for laser line animation
pcount=0;

% initialise states
xtrue= zeros(3,1);
x= zeros(3,1);
P= zeros(3);

% initialise other variables and constants
dt= DT_CONTROLS; % change in time between predicts
dtsum= 0; % change in time since last observation
ftag= 1:size(lm,2); % identifier for each landmark
da_table= zeros(1,size(lm,2)); % data association table 
iwp= 1; % index to first waypoint 
G= 0; % initial steer angle
data= initialise_store(x,P,x); % stored data for off-line
QE= Q; RE= R; if SWITCH_INFLATE_NOISE, QE= 2*Q; RE= 8*R; end % inflate estimated noises (ie, add stabilising noise)
if SWITCH_SEED_RANDOM, randn('state',SWITCH_SEED_RANDOM), end

% main loop 
while iwp ~= 0
    
    % compute true data
    [G,iwp]= compute_steering(xtrue, wp, iwp, AT_WAYPOINT, G, RATEG, MAXG, dt);
    if iwp==0 & NUMBER_LOOPS > 1, iwp=1; NUMBER_LOOPS= NUMBER_LOOPS-1; end % perform loops: if final waypoint reached, go back to first
    xtrue= vehicle_model(xtrue, V,G, WHEELBASE,dt);
    [Vn,Gn]= add_control_noise(V,G,Q, SWITCH_CONTROL_NOISE);
    
    % EKF predict step
    [x,P]= predict (x,P, Vn,Gn,QE, WHEELBASE,dt);
    
    % if heading known, observe heading
    [x,P]= observe_heading(x,P, xtrue(3), SWITCH_HEADING_KNOWN);
    
    % EKF update step
    dtsum= dtsum + dt;
    if dtsum >= DT_OBSERVE
        dtsum= 0;
        [z,ftag_visible]= get_observations(xtrue, lm, ftag, MAX_RANGE);
        z= add_observation_noise(z,R, SWITCH_SENSOR_NOISE);
    
        if SWITCH_ASSOCIATION_KNOWN == 1
            [zf,idf,zn, da_table]= data_associate_known(x,z,ftag_visible, da_table);
        else
            [zf,idf, zn]= data_associate(x,P,z,RE, GATE_REJECT, GATE_AUGMENT); 
        end

        if SWITCH_USE_IEKF == 1
            [x,P]= update_iekf(x,P,zf,RE,idf, 5);
        else
            [x,P]= update(x,P,zf,RE,idf, SWITCH_BATCH_UPDATE); 
        end
        [x,P]= augment(x,P, zn,RE); 
    end
    
    % offline data store
    data= store_data(data, x, P, xtrue);
    
    % plots
    xt= transformtoglobal(veh,xtrue);
    xv= transformtoglobal(veh,x(1:3));
    set(h.xt, 'xdata', xt(1,:), 'ydata', xt(2,:))
    set(h.xv, 'xdata', xv(1,:), 'ydata', xv(2,:))
    set(h.xf, 'xdata', x(4:2:end), 'ydata', x(5:2:end))
    ptmp= make_covariance_ellipses(x(1:3),P(1:3,1:3));
    pcov(:,1:size(ptmp,2))= ptmp;
    if dtsum==0
        set(h.cov, 'xdata', pcov(1,:), 'ydata', pcov(2,:)) 
        pcount= pcount+1;
        if pcount == 15
            set(h.pth, 'xdata', data.path(1,1:data.i), 'ydata', data.path(2,1:data.i))    
            pcount=0;
        end
        if ~isempty(z)
            plines= make_laser_lines (z,x(1:3));
            set(h.obs, 'xdata', plines(1,:), 'ydata', plines(2,:))
            pcov= make_covariance_ellipses(x,P);
        end
    end
    drawnow
end

data= finalise_data(data);
set(h.pth, 'xdata', data.path(1,:), 'ydata', data.path(2,:))    

% 
%

function h= setup_animations()
h.xt= patch(0,0,'b','erasemode','xor'); % vehicle true
h.xv= patch(0,0,'r','erasemode','xor'); % vehicle estimate
h.pth= plot(0,0,'k.','markersize',2,'erasemode','background'); % vehicle path estimate
h.obs= plot(0,0,'r','erasemode','xor'); % observations
h.xf= plot(0,0,'r+','erasemode','xor'); % estimated features
h.cov= plot(0,0,'r','erasemode','xor'); % covariance ellipses

%
%

function p= make_laser_lines (rb,xv)
% compute set of line segments for laser range-bearing measurements
if isempty(rb), p=[]; return, end
len= size(rb,2);
lnes(1,:)= zeros(1,len)+ xv(1);
lnes(2,:)= zeros(1,len)+ xv(2);
lnes(3:4,:)= transformtoglobal([rb(1,:).*cos(rb(2,:)); rb(1,:).*sin(rb(2,:))], xv);
p= line_plot_conversion (lnes);

%
%

function p= make_covariance_ellipses(x,P)
% compute ellipses for plotting state covariances
N= 10;
inc= 2*pi/N;
phi= 0:inc:2*pi;

lenx= length(x);
lenf= (lenx-3)/2;
p= zeros (2,(lenf+1)*(N+2));

ii=1:N+2;
p(:,ii)= make_ellipse(x(1:2), P(1:2,1:2), 2, phi);

ctr= N+3;
for i=1:lenf
    ii= ctr:(ctr+N+1);
    jj= 2+2*i; jj= jj:jj+1;
    
    p(:,ii)= make_ellipse(x(jj), P(jj,jj), 2, phi);
    ctr= ctr+N+2;
end

%
%

function p= make_ellipse(x,P,s, phi)
% make a single 2-D ellipse of s-sigmas over phi angle intervals 
r= sqrtm(P);
a= s*r*[cos(phi); sin(phi)];
p(2,:)= [a(2,:)+x(2) NaN];
p(1,:)= [a(1,:)+x(1) NaN];

%
%

function data= initialise_store(x,P, xtrue)
% offline storage initialisation
data.i=1;
data.path= x;
data.true= xtrue;
data.state(1).x= x;
%data.state(1).P= P;
data.state(1).P= diag(P);

%
%

function data= store_data(data, x, P, xtrue)
% add current data to offline storage
CHUNK= 5000;
if data.i == size(data.path,2) % grow array in chunks to amortise reallocation
    data.path= [data.path zeros(3,CHUNK)];
    data.true= [data.true zeros(3,CHUNK)];
end
i= data.i + 1;
data.i= i;
data.path(:,i)= x(1:3);
data.true(:,i)= xtrue;
data.state(i).x= x;
%data.state(i).P= P;
data.state(i).P= diag(P);

%
%

function data= finalise_data(data)
% offline storage finalisation
data.path= data.path(:,1:data.i);
data.true= data.true(:,1:data.i);
