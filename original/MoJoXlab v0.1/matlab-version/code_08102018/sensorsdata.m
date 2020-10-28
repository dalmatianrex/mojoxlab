function [ pelvis_standing,rthigh_standing, rshank_standing,rfoot_standing] = sensorsdata(tree)
%Body frame quaternions obtained during calibration posture
% [ q_imu_pv_bf,q_imu_th_bf,q_imu_sh_bf,q_imu_ft_bf]
for g=3:size(tree.subject.frames.frame,2)
    
    if size(tree.subject.frames.frame(g).sensorAcceleration,1)>1
        disp(' ')
        disp('**WARNING - Please amend code**')
    end
    
    for sens=1:10
        
        if strcmp(tree.subject.sensors.sensor(sens).label,'Pelvis')==1
            SegmentID='Pelvis';
            side=1;
        elseif strcmp(tree.subject.sensors.sensor(sens).label,'LeftUpperLeg')==1
            side=1;
            SegmentID='Thigh';
        elseif strcmp(tree.subject.sensors.sensor(sens).label,'RightUpperLeg')==1
            side=2;
            SegmentID='Thigh';
        elseif strcmp(tree.subject.sensors.sensor(sens).label,'LeftLowerLeg')==1
            side=1;
            SegmentID='Shank';
        elseif strcmp(tree.subject.sensors.sensor(sens).label,'RightLowerLeg')==1
            side=2;
            SegmentID='Shank';
        elseif strcmp(tree.subject.sensors.sensor(sens).label,'LeftFoot')==1
            side=1;
            SegmentID='Foot';
        elseif strcmp(tree.subject.sensors.sensor(sens).label,'RightFoot')==1
            side=2;
            SegmentID='Foot';
        elseif strcmp(tree.subject.sensors.sensor(sens).label,'prop')==1
            %No action
        else
            disp(' ')
            disp(['**WARNING: Sensor ' num2str(sens) ' is called ' tree.subject.sensors.sensor(sens).label '. Please amend the script**'])
        end
        
        if exist('SegmentID','var')>0
            
            
            eval([SegmentID '_Orientation{1,side}(g-2,1)=tree.subject.frames.frame(g).sensorOrientation(:,(sens*4)-3);'])
            eval([SegmentID '_Orientation{1,side}(g-2,2)=tree.subject.frames.frame(g).sensorOrientation(:,(sens*4)-2);'])
            eval([SegmentID '_Orientation{1,side}(g-2,3)=tree.subject.frames.frame(g).sensorOrientation(:,(sens*4)-1);'])
            eval([SegmentID '_Orientation{1,side}(g-2,4)=tree.subject.frames.frame(g).sensorOrientation(:,(sens*4));'])
            
            
            
            clear SegmentID side
        end
    end %end of for sens=1:10
end

Wn=60/2; % MVN sampling rate was 60 Hz
cutoff=6;
[b,a] = butter(2,cutoff/Wn,'low');


for side=1:2 %1:Left ; 2: Right
    
    pelvis_standing = filtfilt(b,a,(Pelvis_Orientation{1,1}));
    rthigh_standing{side} = filtfilt(b,a,(Thigh_Orientation{1,side}));
    rshank_standing {side} = filtfilt(b,a,(Shank_Orientation{1,side}));
    rfoot_standing {side}  = filtfilt(b,a,(Foot_Orientation{1,side}));
    
    
%     q_imu_f_pv0 = median(pelvis_standing(5:350,:))';
%     q_imu_f_th0{side} = median(rthigh_standing{1,side}(:,:))';
%     q_imu_f_sh0{side}= median(rshank_standing{1,side}(:,:))';
%     q_imu_f_ft0{side} = median(rfoot_standing{1,side}(:,:))';
end
end
