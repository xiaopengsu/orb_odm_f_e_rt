clear all;clc;close all;
% GPStimestamp,northing,easting,Lat,Lon,Hgt,speed,Heading,Roll,Pitch
f_traj=load('dtl_image_rt.txt');%
%% 显示轨迹
f_t=f_traj(1:size(f_traj,1),2:4);
f_r=f_traj(1:size(f_traj,1),5:13);
f_tt=[];
% f_tt(1,:)=f_t(1,:);
f_tt(1,:)=f_t(1,:); %f_tt(1,:)=-f_t(1,:);
for i=2:size(f_traj,1)
%     f_t(i,1)^2+f_t(i,2)^2+f_t(i,3)^2 %尺度都为1
%     if(0==f_traj(i,18))
%         i % continue
% %         f_t(i,:)=f_t(i-1,:)
%     end

%     if(f_t(i,3)<0)
%         f_t(i,:)=-f_t(i,:);
%     end
    f_rr=reshape(f_r(i,:),3,3);    
%     f_tt(i,:)=f_tt(i-1,:)+(f_rr*f_t(i,:)')';
    f_tt(i,:)=(f_rr*f_tt(i-1,:)')'+f_t(i,:);
end
figure(1)
plot3(f_tt(:,1),f_tt(:,2),f_tt(:,3),'r*')
xlabel('估计测量x距离')
ylabel('估计测量y距离')
zlabel('估计测量z距离')
legend('tj')
title('traj')