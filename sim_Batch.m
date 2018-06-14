%sim_Batch.m Script to perform batch quadrotor simulation
%   Author: Fiona Chui (fiona.chui@mail.mcgill.ca)
%   Last Updated: December 12, 2016
%   Description: Script to perform batch quadrotor simulation.
%                Initial collision conditions are prescribed at different
%                intervals for each variable. However, prescribed initial
%                conditions will likely not be perfectly achieved, as the
%                simulation is dynamic. For validation, using sim_MonteCarlo.m
%                is probably better. 
%-------------------------------------------------------------------------%

Batch = [];
iBatch = 0;
crash_array = [];

for vImpact = 1:0.2:3
% for vImpact = 0:0.05:4
    for inclinationImpact = 10:2:35
%  inclinationImpact =20;
        yawImpact = 225;
                angle = (inclinationImpact - 0.0042477)/1.3836686;
                rollImpact = angle;
                pitchImpact = angle;
           
            
            iBatch = iBatch + 1; 
            disp(iBatch)

            [CrashData.ImpactIdentification,CrashData.FuzzyInfo,CrashData.Plot,CrashData.timeImpact] = startsim(vImpact, rollImpact, pitchImpact, yawImpact,iBatch);
            
            CrashData.roll_atImpact = rollImpact;
            CrashData.pitch_atImpact = pitchImpact; %in degrees;
            CrashData.vel_atImpact = vImpact ;
            CrashData.inclinationImpact = inclinationImpact;
            Batch = [Batch;CrashData];
            
            crash_array(iBatch) = 0;
            if CrashData.Plot.posns(3,end) <=0 || (0>CrashData.Plot.posns(1,end) && CrashData.Plot.posns(1,end)>2) || CrashData.Plot.posns(2,end)>2 
%                 if CrashData.Plot.times(end) - CrashData.timeImpact <= 0.9
                    crash_array(iBatch) = 1;
%                 end
            end
            
      %%
   
           
        
    end
end
% save('test5-23.mat')

save('Batch_v(0-0.05-4)I(-10-1-35).mat')
% roll_atImpact
% roll=cell2mat(Batch.roll_atImpact)
%%
temp=struct2cell(Batch);
xx=[temp{8,:}];
yy=[temp{7,:}];
%  [X,Y]=meshgrid(xx,yy);
%  plotmatrix(X,Y)

for i=1:length(crash_array)
    if crash_array(i)==0
        cc(i,:)=[0 0 1]
    else
        cc(i,:)=[1 0 0]
    end
end
% ccc(C==0,:)=[0 0 1];
% ccc(C==1,:)=[1 0 0];
scatter(xx,yy,50,cc,'filled')
xlabel('Inclination')
ylabel('VelocityImpact')