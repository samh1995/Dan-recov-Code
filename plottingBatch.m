
temp=struct2cell(Batch);
xx=[temp{8,:}];
yy=[temp{7,:}];

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