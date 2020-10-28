# OU-Cardiff

These are the files that Mohammed has provided me:

* BFtoGF.m - source unknown
* complexConjugateQuaternion.m - source unknown
* correctionQuaternion.m - source unknown
* DCM.m - source unknown
* e2B.m - source unknown
* GFtoBF.m - source unknown
* JointAngles_IMU_MA.m - MOHAMMAD AL-AMRI custom code
* load_mvnx.m - XSens
* multiplicationQuaternions.m - source unknown
* RI_MohammadFunction.m - Riasat's modificaiton on top of MOHAMMAD AL-AMRI's code
* sensorsdata.m - MOHAMMAD AL-AMRI custom code
* StandingCalib_IMU_MA.m - MOHAMMAD AL-AMRI custom code
* UseFunction_Data_Mohammad_Test.m - MOHAMMAD AL-AMRI custom code __MAIN__
* UseFunction_Data_Mohammad_Test11.m - MOHAMMAD AL-AMRI custom code


Trying to unpack UseFunction_Data_Mohammad_Test.m code:

Takes 2 file inputs:
1. Static calibration mvnx file
2. Activity mvnx file

Loads mvnx file and generates a data tree for each mvnx file

data tree is passed on to "sensorsdata" function. And generates for variables (d1, d2, d3, d4) - most probably data in quaternions for static calibration

and (d5, d6, d7, d8) - most probably data in quaternions for activity

Actually after close examination of sensorsdata.m,

d1 - d4 and d5 - d8 is:
pelvis_standing,rthigh_standing, rshank_standing,rfoot_standing

we will return to sensorsdata function later on

then pass d1 - d4 to StandingCalib_IMU_MA function along with Stance Start and Stance End values (this is just taking a certain portion of the data). Standing Calib function outputs the following variables:
q_imu_pv_bf,q_imu_th_bf,q_imu_sh_bf,q_imu_ft_bf

where,
q is quaternions
imu is imu
pv is pelvis
th is thigh
sh is shank
ft is foot
bf is body frame

Then 
q_imu_pv_bf,q_imu_th_bf,q_imu_sh_bf,q_imu_ft_bf

and d5 - d8 is passed on to JointAngles_IMU_MA function that outputs the joint angles (Hip, Knee and Ankle for two sides in 3/2 planes)

All the calculations to generate joint angles are done within JointAngles_IMU_MA function


while trying to extract sensor orientation values from mvnx files,
they are sequenced for each frame across all sensors.

so, for example,
if i want to get pelvis:

tree.subject.frames.frame(l).sensorOrientation(1:4)
where, l is the iterator to go over all the frames

as we are using 7 sensors, so we need 7 * 4 = 28 vectors for each input (calibration and activity). There are 2 such inputs so in total 28 * 2 = 56 vectors as input

This is the sequence of sensors:
'Pelvis'
'RightUpperLeg' - RT Right Thigh
'RightLowerLeg' - RS Right Shank
'RightFoot' - RF Right Foot
'LeftUpperLeg' - LT Left Thigh
'LeftLowerLeg' - LS Left Shank
'LeftFoot' - LF Left Foot


Understanding sensorsdata.m

So, 
eval([SegmentID '_Orientation{1,side}(g-2,1)=tree.subject.frames.frame(g).sensorOrientation(:,(sens*4)-3);'])
eval([SegmentID '_Orientation{1,side}(g-2,2)=tree.subject.frames.frame(g).sensorOrientation(:,(sens*4)-2);'])
eval([SegmentID '_Orientation{1,side}(g-2,3)=tree.subject.frames.frame(g).sensorOrientation(:,(sens*4)-1);'])
eval([SegmentID '_Orientation{1,side}(g-2,4)=tree.subject.frames.frame(g).sensorOrientation(:,(sens*4));'])

these statements generate 4 variables:
Pelvis_Orientation 
Thigh_Orientation (with left and right sides)
Shank_Orientation (with left and right sides)
Foot_Orientation (with left and right sides)

Then, a filter is designed:

Wn=60/2; % MVN sampling rate was 60 Hz
cutoff=6;
[b,a] = butter(2,cutoff/Wn,'low');

where, b and a are the transfer function co-efficients

   pelvis_standing = filtfilt(b,a,(Pelvis_Orientation{1,1}));
    rthigh_standing{side} = filtfilt(b,a,(Thigh_Orientation{1,side}));
    rshank_standing {side} = filtfilt(b,a,(Shank_Orientation{1,side}));
    rfoot_standing {side}  = filtfilt(b,a,(Foot_Orientation{1,side}))

    then filtering the data and saving them to corresponding variables.

    in reality these are the variables that are going to act as input to the main algorithm after being filtered:
    pelvis_standing, rthigh_standing{side}, rshank_standing {side}, rfoot_standing {side}

    where, side is the variable indicating left(1) and right(2) side of the bodies 


    Finally:

    In RI_MohammadFunction.m

    for side=1:2
    
    [q_imu_pv_bf,q_imu_th_bf,q_imu_sh_bf,q_imu_ft_bf] = StandingCalib_IMU_MA(d1,d2{1,side},d3{1,side},d4{1,side},5,50);
    
    [Ahip_angle{side},Aknee_angle{side},Aankle_angle{side}]= JointAngles_IMU_MA(q_imu_pv_bf,q_imu_th_bf,q_imu_sh_bf,q_imu_ft_bf,d5,d6{1,side},d7{1,side},d8{1,side},1);
    
    clearvars -except calib d1 d2 d3 d4 d5 d6 d7 d8 Ahip_angle Aknee_angle Aankle_angle fileStatic fileTrial
end

These are the lines that do the trick.

Here:
d1 - d4 and d5 - d8 are the filtered orientations (pelvis, thigh, shank and foot) for static and activity respectively

