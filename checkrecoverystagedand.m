function [Control] = checkrecoverystagedand(Pose, ImpactParams, Control, ImpactInfo,timenow,timeImapct)
%checkrecoverystage.m  Checks recovery stage for dand controller
%%%%% CONDITIONS %%%%%

    % Stage 2 condition
 
  safedistance=ImpactParams.wallLoc-Pose.posn(1)> 0.4 ||  timenow - timeImapct>0.2 ;
%  ImpactParams.wallLoc-Pose.posn(1)> 0.2  || timenow - timeImapct>0.2;
  
  
 
    % Stage 0: pre-impact
    % Stage 1: impact detected
    % Stage 2: cleared a certain distance from the wall activate dand acc
    % position
   
    
%%%%% CHECK LOGIC %%%%%
    switch Control.recoveryStage
        case 0
            if ImpactInfo.firstImpactDetected
                Control.recoveryStage = 1;
            end
        case 1
            if safedistance
                Control.recoveryStage = 2;
            end
       
    end  
end