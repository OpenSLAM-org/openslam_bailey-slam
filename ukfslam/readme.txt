UKF-SLAM
--------

This set of Matlab functions performs a simulation of simultaneous
localisation and mapping (SLAM) using the unscented Kalman filter
(UKF). It is an adaptation of the EKF-SLAM simulator also
available online.

To use this simulator, you need to first download my "Matlab
Utilities" package and add it to your Matlab path. These utilities
perform all kinds of filtering operations, and are fast, being
completely vectored for speed. They are available from:
http://www.acfr.usyd.edu.au/homepages/academic/tbailey/software/software.html

To run the simulator, first load a map, then run the simulator.
Eg,

  load example_webmap
  data = ukfslam_sim(lm, wp);

The returned "data" structure contained recorded values for
subsequent off-line processing, if desired.

To configure the simulator, edit the file "configfile.m". This
file contains process and observation noise settings and various
other settings.

Tim Bailey 2006.
