function [desiredaccel] = calculatedesacceleration(Pose, Twist)
%calculaterefacceleration.m Calculate a_ref for Recovery Controller
desrdposn=[0;0;2]; %%desired posn
desiredVinertial=[0;0;0]; %%desired vel
%-------------------------------------------------------------------------%
% global damping_ratio nat_freq 
damping_ratio=0.7;
nat_freq=1;

errouter_d =[Pose.posn(1)-desrdposn(1);Pose.posn(2)-desrdposn(2);Pose.posn(3)-desrdposn(3)];
errouter_d_dot=[Twist.posnDeriv(1)-desiredVinertial(1);Twist.posnDeriv(2)-desiredVinertial(2);Twist.posnDeriv(3)-desiredVinertial(3)];
desiredaccel = -2*damping_ratio*nat_freq*errouter_d_dot-(nat_freq)^2*errouter_d;  %inertial desired accelrraion 
% Control.acc=desiredaccel;


end