function [Hip_angle,Knee_angle,Ankle_angle] = JointAngles_IMU_MA(q_imu_pv_bf,q_imu_th_bf, q_imu_sh_bf,q_imu_ft_bf,Pelvis_Orientation,Thigh_Orientation,Shank_Orientation,Foot_Orientation,side)
% 
% Pelvis_Orientation=Xsens_Pelvis_SensorOrientation_Quat{subj,task}{session,rater}{1,1};
% Thigh_Orientation=Xsens_UpperLeg_SensorOrientation_Quat{subj,task}{session,rater}{1,side};
% Shank_Orientation=Xsens_LowerLeg_SensorOrientation_Quat{subj,task}{session,rater}{1,side};
% Foot_Orientation=Xsens_Foot_SensorOrientation_Quat{subj,task}{session,rater}{1,side};

%Written by MOhammad Al-Amri, Cardiff University, in August 2017

%Script requires sensor orientation data from and outputs joint angles

%Joint kinematics are based on Joint Coordinate System (JCS) as proposed by
%Grood and Suntay (1983)

%--------------------------------------------------------------------------
%Initial Quaternion Definition body quaternions during calibration posture
i = [1 0 0];%unit vector in direction of the x axis.
j = [0 1 0];%unit vector in direction of the y axis.
k = [0 0 1];%unit vector in direction of the z axis.

for x=1:size(Pelvis_Orientation,1);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % body to global orientation
    q_bf_pv (x,:) = BFtoGF(Pelvis_Orientation(x,:), q_imu_pv_bf);
    q_bf_th(x,:) = BFtoGF(Thigh_Orientation(x,:), q_imu_th_bf);
    q_bf_sh (x,:)= BFtoGF(Shank_Orientation(x,:), q_imu_sh_bf);
    if isempty(q_imu_ft_bf)==0
        q_bf_ft(x,:) = BFtoGF(Foot_Orientation(x,:), q_imu_ft_bf);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Floating axis of each Hip(h),Knee(k), and Ankle (a)
    e2_h(:,x) = e2B(DCM(q_bf_th(x,:))*i', -DCM(q_bf_pv(x,:))*j');%This is based on the JCS proposed by Wu et al (2002)
    e2_k(:,x) = e2B(DCM(q_bf_sh(x,:))*i', DCM(q_bf_th(x,:))*k');% this is based on JCS proposed by Grood and Suntay (1983)
    if isempty(q_imu_ft_bf)==0
        e2_a(:,x) = e2B(DCM(q_bf_ft(x,:))*k', DCM(q_bf_sh(x,:))*k');%This is based on the JCS proposed by Wu et al (2002)
    end
    %++++++++++++++++++++++++++++++++++++++++++++ 
    % Left Joint Angles 
    %++++++++++++++++++++++++++++++++++++++++++++
    if side==1
        %Hip Joint rotations
        alpa_hip(x) = asin(dot(e2_h(:,x), DCM(q_bf_pv(x,:))*i'));%sagittal plane
        beta_hip(x) = pi/2-acos(dot(-DCM(q_bf_pv(x,:))*j', DCM(q_bf_th(x,:))*i'));%frontal plane
        gama_hip(x) = asin(dot(e2_h(:,x), DCM(q_bf_th(x,:))*k'));%transverse plane
        alpha_angle_hip(x) = radtodeg(alpa_hip(x));
        beta_angle_hip(x) = radtodeg(beta_hip(x));
        gama_angle_hip(x) = radtodeg(gama_hip(x));

        %Knee Joint rotations
        alpa_knee (x) = -asin(dot(e2_k (:,x), DCM(q_bf_th(x,:))*i'));%sagittal plane
        beta_knee (x)= pi/2-acos(dot(DCM(q_bf_th(x,:))*k', DCM(q_bf_sh(x,:))*i'));%frontal plane
        gama_knee (x) = asin(dot(e2_k(:,x), DCM(q_bf_sh(x,:))*k'));
        alpha_angle_knee  (x) = radtodeg(alpa_knee(x));
        beta_angle_knee (x) = radtodeg(beta_knee(x));
        gama_angle_knee (x) = radtodeg(gama_knee(x));

        %Ankle Joint rotations
        if isempty(q_imu_ft_bf)==0
            alpa_ankle (x) = asin(dot(e2_a(:,x), DCM(q_bf_sh(x,:))*i'));%sagittal plane
            beta_ankle (x) = pi/2-acos(dot(DCM(q_bf_sh(x,:))*k', DCM(q_bf_ft(x,:))*k'));%frontal plane
            alpha_angle_ankle (x) = radtodeg(alpa_ankle(x));
            beta_angle_ankle (x) = radtodeg(beta_ankle (x));
        end

    %++++++++++++++++++++++++++++++++++++++++++++ 
    % Right Joint Angles 
    %++++++++++++++++++++++++++++++++++++++++++++
    elseif side==2
        %Hip Joint rotations
        alpa_hip(x) = asin(dot(e2_h(:,x), DCM(q_bf_pv(x,:))*i'));%sagittal plane
        beta_hip(x) = acos(dot(-DCM(q_bf_pv(x,:))*j', DCM(q_bf_th(x,:))*i'))-pi/2;%frontal plane
        gama_hip(x) = asin(dot(e2_h(:,x), DCM(q_bf_th(x,:))*k'));%transverse plane
        alpha_angle_hip(x) = radtodeg(alpa_hip(x));
        beta_angle_hip(x) = radtodeg(beta_hip(x));
        gama_angle_hip(x) = radtodeg(gama_hip(x));

        %Knee Joint rotations
        alpa_knee (x) = -asin(dot(e2_k (:,x), DCM(q_bf_th(x,:))*i'));%sagittal plane
        beta_knee (x)= acos(dot(DCM(q_bf_th(x,:))*k', DCM(q_bf_sh(x,:))*i'))-pi/2;%frontal plane
        gama_knee (x) = asin(dot(e2_k(:,x), DCM(q_bf_sh(x,:))*k'));%transverse plane
        alpha_angle_knee  (x) = radtodeg(alpa_knee(x));
        beta_angle_knee (x) = radtodeg(beta_knee(x));
        gama_angle_knee (x) = radtodeg(gama_knee(x));

        %Ankle Joint rotations
        if isempty(q_imu_ft_bf)==0
            alpa_ankle (x) = asin(dot(e2_a(:,x), DCM(q_bf_sh(x,:))*i'));%sagittal plane
            beta_ankle (x) = acos(dot(DCM(q_bf_sh(x,:))*k', DCM(q_bf_ft(x,:))*k'))-pi/2;%frontal plane
            alpha_angle_ankle (x) = radtodeg(alpa_ankle(x));
            beta_angle_ankle (x) = radtodeg(beta_ankle (x));
        end
    end %end of for if side=1 or 2
end %end of for each row of data
Hip_angle=[alpha_angle_hip; beta_angle_hip; gama_angle_hip]';
Knee_angle=[alpha_angle_knee; beta_angle_knee; gama_angle_knee]';

Ankle_angle=[alpha_angle_ankle; beta_angle_ankle]';   

end
       