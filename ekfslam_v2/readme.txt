EKF-SLAM Simulator (version 2.0)
------------------

This simulator demonstrates a simple implementation of
standard EKF-SLAM. It permits simple configuration via 
'configfile.m' to perform SLAM with various control parameters,
noises, etc. Also various switches are available to choose
known data-association versus gating, etc.


The key file in this simulator is called 'ekfslam_sim.m'. Type
'help ekfslam_sim' for more information of how to use it.

In addition to on-line animations, the simulator returns a
data-structure of the logged state information for off-line
processing. An example use of this data is shown in m-file
'plot_feature_loci.m', which plots the trajectories of the 
landmark estimates.

Tim Bailey and Juan Nieto
2004.


Note on Global Variables
------------------------

This version of the simulator uses global variables for 
all large objects, such as the state covariance matrix.
While bad programming practice, it is a necessary evil
for MatLab efficiency, as MatLab has no facility to avoid
gratuitous memory allocation and copying when passing
(and modifying) variables between functions. With this
concession, effort has been made to keep the code as 
clean and modular as possible.
