SLAM Simulations

These MatLab simulations are of EKF-SLAM, FastSLAM 1.0, FastSLAM 2.0
and UKF-SLAM. The intent of these simulators was to permit comparison
of the different map building algorithms. However, they might also be
useful to the wider research community interested in SLAM, as a
straight-forward implementation of the algorithms.

EKF-SLAM version 1. This older version of the EKF-SLAM simulator is
probably easier to understand than the 2nd version, as it avoids using
global variables.

EKF-SLAM version 2. This version of the EKF-SLAM simulator runs much
faster in MatLab as it avoids copying overhead by using global
variables. It also comes with an alternative observation model that
can replace the 'update' function in 'ekfslam_sim.m'. This alternative
is a "global constraint" model devised by Jose Guivant, and may have
better linearisation properties than the conventional range-bearing
model.

FastSLAM 1.0 and 2.0. This implementation is slow in Matlab due to the
overhead of looping constructs etc. However, it can give a good idea
of how each algorithm works, and may serve as a starting point for
more efficient implementations.

UKF-SLAM. This simulator is a direct adaptation of the EKF-SLAM code,
but replaces the EKF with an unscented Kalman filter (UKF). To run
this code you must first install MatLab utilities.

