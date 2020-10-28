function [Aankle_angle, Ahip_angle, Aknee_angle] = RI_MohammadFunction(fileStatic, fileTrial)
%% This script has been slightly modified by RI, to to read Static trial filenames and Actual Trial file names (walking, jumping, squat, etc)...
%   and output Ankle angles, Hip angles and Knee angles

% clear
%clc

com=1;

if com==1 %EGH
    addpath ('/')
elseif com==2 %TDS
    addpath ('/')
end

calib=1;
for i=1:2
    
    
    
    if calib ==1
        
        %filename=(['C:\Users\whcma5\OneDrive - Cardiff University\MohammadsWork\Research\Research Projects\Xsens-Validation study\MatlabCodes\IMU\Example_Data\Subj2-standing'])
        %filename=(['../data/Subj1/Session1/Exp/Trial-002#My MVN System.mvnx']);
        filename = fileStatic;
        
        %          filename=(['C:\Users\Mohammed\OneDrive - Cardiff University\MohammadsWork\Research\Research Projects\Xsens-Validation study\MatlabCodes\IMU\Example_Data\Subj2-standing'])%EGH
        
    else
        
        % filename=(['C:\Users\whcma5\OneDrive - Cardiff University\MohammadsWork\Research\Research Projects\Xsens-Validation study\MatlabCodes\IMU\Example_Data\Subj2-Squat5'])
        %          filename=(['C:\Users\Mohammed\OneDrive - Cardiff University\MohammadsWork\Research\Research Projects\Xsens-Validation study\MatlabCodes\IMU\Example_Data\Subj2-Squat5'])
        %          filename=(['C:\Users\Mohammed\OneDrive - Cardiff University\MohammadsWork\Research\Research Projects\Xsens-Validation study\MatlabCodes\IMU\Example_Data\Subj2-Squat5'])%EGH
        %filename=(['../data/Subj1/Session1/Exp/Trial-008#My MVN System.mvnx']);
        filename = fileTrial;
        
    end
    % load data
    tree = load_mvnx(filename);
    % read some basic data from the file
    mvnxVersion = tree;
    fileComments = tree.subject.comment;
    
    for g=3:size(tree.subject.frames.frame,2)
        
        if size(tree.subject.frames.frame(g).sensorAcceleration,1)>1
            disp(' ')
            disp('**WARNING - Please amend code**')
        end
        
    end
    
    
    %%
    if calib ==1
        %%
        [d1,d2,d3,d4] =sensorsdata(tree);
        %         [d11,d21,d31,d41] =StandingCalib_IMU_MA(tree);
        
        calib=2;
        disp(' Mohammed function running calibration')
        clearvars -except calib d1 d2 d3 d4 fileStatic fileTrial
        %%
        % walking
    else
        
        [d5,d6,d7,d8] =sensorsdata(tree);
        clearvars -except calib d1 d2 d3 d4 d5 d6 d7 d8 fileStatic fileTrial
        
        
        disp(' Mohammad function running main trial')
    end
end
clear side
%cd('C:\Users\mohammed\OneDrive - Cardiff University\MohammadsWork\Research\Research Projects\Xsens-Validation study\MatlabCodes\IMU\IMUCalibrationMethod\OU')
for side=1:2
    
    [q_imu_pv_bf,q_imu_th_bf,q_imu_sh_bf,q_imu_ft_bf] = StandingCalib_IMU_MA(d1,d2{1,side},d3{1,side},d4{1,side},5,50);
    
    [Ahip_angle{side},Aknee_angle{side},Aankle_angle{side}]= JointAngles_IMU_MA(q_imu_pv_bf,q_imu_th_bf,q_imu_sh_bf,q_imu_ft_bf,d5,d6{1,side},d7{1,side},d8{1,side},1);
    
    clearvars -except calib d1 d2 d3 d4 d5 d6 d7 d8 Ahip_angle Aknee_angle Aankle_angle fileStatic fileTrial
end

%plot(Aknee_angle{1,2}(:,1));hold on;plot(Aknee_angle{1,1}(:,1),'r')
end