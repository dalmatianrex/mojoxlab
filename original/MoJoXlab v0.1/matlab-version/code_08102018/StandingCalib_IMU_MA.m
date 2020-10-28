function [q_imu_pv_bf,q_imu_th_bf,q_imu_sh_bf,q_imu_ft_bf] = StandingCalib_IMU_MA(Pelvis_Orientation,Thigh_Orientation,Shank_Orientation,Foot_Orientation,StanceStart,StanceEnd)

%Written by MOhammad Al-Amri, Cardiff University, in August 2017

%Input: sensor orientation data from a period of quiet stance. Orientation
%data are required from sensors placed on the pelvis, thigh and shank.
%Orientation data are optional from sensors placed on the foot. The period
%of quiet stance is defined by StartStance and EndStance.

%Output: rotation matrices to calibrate sensor to segment orientation

%--------------------------------------------------------------------------

%Body frame quaternions obtained during calibration posture
% [ q_imu_pv_bf,q_imu_th_bf,q_imu_sh_bf,q_imu_ft_bf]

  
q_imu_f_pv0 = median(Pelvis_Orientation(StanceStart:StanceEnd,:))';
q_imu_f_th0 = median(Thigh_Orientation(StanceStart:StanceEnd,:))';
q_imu_f_sh0= median(Shank_Orientation(StanceStart:StanceEnd,:))';

q_imu_f_ft0= median(Foot_Orientation(StanceStart:StanceEnd,:))';


%--------------------------------------------------------------------------
%Calculate the body-to-sensor rotation matrix

%pelvis

%convert from unit quaternion to Direction Cosine Matrix
qc_pv = correctionQuaternion(q_imu_f_pv0);% This correction will align the pelvis sensor with the gravity

%Initial Quaternion Definition body quaternions during calibration posture
i = [1 0 0];%unit vector in direction of the x axis.
j = [0 1 0];%unit vector in direction of the y axis.
k = [0 0 1];%unit vector in direction of the z axis.

rad_90  = degtorad(90);
rad_180 = degtorad(180);
q_rot90  = transpose([cos(rad_90/2) 1*sin(rad_90/2) 0 0]);%q(90,i)
q_rot180 = transpose([cos(rad_180/2) (sqrt(2)/2)*sin(rad_180/2) 0 (sqrt(2)/2)*sin(rad_180/2)]);%q(180,[sqrt(2)/2,0,sqrt(2)/2)])

%The body frame of the pelvis with respect to the global frame during the initial posture
q_bf_pv0 = multiplicationQuaternions(qc_pv,q_imu_f_pv0);

%Definition of body frame of the thigh
q_bf_th0 = multiplicationQuaternions(q_bf_pv0, q_rot90);

%Definition of body frame of the shank
q_bf_sh0 = q_bf_th0;

%Definition of body frame of the foot

    q_bf_ft0 = multiplicationQuaternions(q_bf_sh0, q_rot180);


% sensor-to-body orientation (Equation (7))
q_imu_pv_bf = GFtoBF(q_bf_pv0, q_imu_f_pv0);% pelvis
q_imu_th_bf = GFtoBF(q_bf_th0, q_imu_f_th0);% thigh
q_imu_sh_bf = GFtoBF(q_bf_sh0, q_imu_f_sh0);% shank
q_imu_ft_bf = GFtoBF(q_bf_ft0, q_imu_f_ft0);%foot

end