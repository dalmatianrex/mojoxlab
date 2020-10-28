%clear
%clc

% com=2;
% 
% if com==1 %EGH
%   addpath (' C:\Users\Mohammad\OneDrive - Cardiff University\MohammadsWork\Research\Research Projects\Xsens-Validation study\MatlabCodes\IMU\IMUCalibrationMethod')
% elseif com==2 %TDS
%     addpath ('C:\Users\mohammed\OneDrive - Cardiff University\MohammadsWork\Research\Research Projects\Xsens-Validation study\MatlabCodes\IMU')
% end

calib=1;
% for i=1:2
% 
%     
% 
%     if calib ==1
%         
%         %filename=(['C:\Users\whcma5\OneDrive - Cardiff University\MohammadsWork\Research\Research Projects\Xsens-Validation study\MatlabCodes\IMU\Example_Data\Subj2-standing'])
%         filename=(['C:\Users\mohammed\OneDrive - Cardiff University\MohammadsWork\Research\Research Projects\Xsens-Validation study\MatlabCodes\IMU\Example_Data\Subj2-standing']);
%   
% %          filename=(['C:\Users\Mohammed\OneDrive - Cardiff University\MohammadsWork\Research\Research Projects\Xsens-Validation study\MatlabCodes\IMU\Example_Data\Subj2-standing'])%EGH
%         
%     else
%    
%        % filename=(['C:\Users\whcma5\OneDrive - Cardiff University\MohammadsWork\Research\Research Projects\Xsens-Validation study\MatlabCodes\IMU\Example_Data\Subj2-Squat5'])
% %          filename=(['C:\Users\Mohammed\OneDrive - Cardiff University\MohammadsWork\Research\Research Projects\Xsens-Validation study\MatlabCodes\IMU\Example_Data\Subj2-Squat5'])
% %          filename=(['C:\Users\Mohammed\OneDrive - Cardiff University\MohammadsWork\Research\Research Projects\Xsens-Validation study\MatlabCodes\IMU\Example_Data\Subj2-Squat5'])%EGH
%  filename=(['C:\Users\mohammed\OneDrive - Cardiff University\MohammadsWork\Research\Research Projects\Xsens-Validation study\MatlabCodes\IMU\Example_Data\Subj2-walking8']);
%         
%     end
%     % load data
%     tree = load_mvnx(filename);
%     % read some basic data from the file
%     mvnxVersion = tree;
%     fileComments = tree.subject.comment;
%     
%     for g=3:size(tree.subject.frames.frame,2)
%         
%         if size(tree.subject.frames.frame(g).sensorAcceleration,1)>1
%             disp(' ')
%             disp('**WARNING - Please amend code**')
%         end
%         
%     end
    
    
    %%
    if calib ==1
        %%
        [d1,d2,d3,d4] =sensorsdata(tree);
        
        calib=2;
        disp(' Test 1 ')  
        clearvars -except calib d1 d2 d3 d4  
        %%
        % walking
    else
        
        [d5,d6,d7,d8] =sensorsdata(tree);
        clearvars -except calib d1 d2 d3 d4 d5 d6 d7 d8
        
        
        disp(' Test ')      
    end
end   
    clear side
    for side=1:1
        side=1
       [Ahip_angle,Aknee_angle,Aankle_angle]= JointAngles_IMU_MA(d5,d6{side,:},d7{side,:},d8{side,:},d1,d2{side,:},d3{side,:},d4{side,:},1)
       
%        (q_imu_pv_bf,q_imu_th_bf, q_imu_sh_bf,q_imu_ft_bf,Pelvis_Orientation,Thigh_Orientation,Shank_Orientation,Foot_Orientation,side)
        
%         [Ahip_angle,Aknee_angle,Aankle_angle]= JointAngles_IMU(tree,d1,d2,d3,d4);
        
    end
    
    
