function [desiredaccel] = calculatedesacceleration(Pose, Twist, Control,ImpactParams)
%calculaterefacceleration.m Calculate a_ref for Recovery Controller


 switch Control.recoveryStage
     
     
     case 1
        desiredaccel=Control.accelRef;
        
     case 2 
         
%         desrdposn=[ImpactParams.wallLoc-1;0;10]; %%desired posn
        desrdposn=[1;1;10]; %%desired posn

        desiredVinertial=[0;0;0]; %%desired vel

        damping_ratio=0.7;
        nat_freq=1;

         errouter_d =[Pose.posn(1)-desrdposn(1);Pose.posn(2)-desrdposn(2);Pose.posn(3)-desrdposn(3)];
         errouter_d_dot=[Twist.posnDeriv(1)-desiredVinertial(1);Twist.posnDeriv(2)-desiredVinertial(2);Twist.posnDeriv(3)-desiredVinertial(3)];
         desiredaccel = -2*damping_ratio*nat_freq*errouter_d_dot-(nat_freq)^2*errouter_d;  %inertial desired accelrraion 

 end 

end