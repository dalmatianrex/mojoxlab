%% Input Data
staticQuaternions = csvread('staticQuaternions.csv');
walkingQuaternions = csvread('walkingQuaternions.csv');

staticPelvis_Orientation{1} = staticQuaternions(:,1:4);
staticThigh_Orientation{1} = staticQuaternions(:,17:20);
staticThigh_Orientation{2} = staticQuaternions(:,5:8);
staticShank_Orientation{1} = staticQuaternions(:,21:24);
staticShank_Orientation{2} = staticQuaternions(:,9:12);
staticFoot_Orientation{1} = staticQuaternions(:,25:28);
staticFoot_Orientation{2} = staticQuaternions(:,13:16);

walkingPelvis_Orientation{1} = walkingQuaternions(:,1:4);
walkingThigh_Orientation{1} = walkingQuaternions(:,17:20);
walkingThigh_Orientation{2} = walkingQuaternions(:,5:8);
walkingShank_Orientation{1} = walkingQuaternions(:,21:24);
walkingShank_Orientation{2} = walkingQuaternions(:,9:12);
walkingFoot_Orientation{1} = walkingQuaternions(:,25:28);
walkingFoot_Orientation{2} = walkingQuaternions(:,13:16);

%% Filter data

Wn=60/2; % MVN sampling rate was 60 Hz
cutoff=6;
[b,a] = butter(2,cutoff/Wn,'low');

for side=1:2 %1:Left ; 2: Right
    
    static_pelvis_standing = filtfilt(b,a,(staticPelvis_Orientation{1,1}));
    static_rthigh_standing{side} = filtfilt(b,a,(staticThigh_Orientation{1,side}));
    static_rshank_standing {side} = filtfilt(b,a,(staticShank_Orientation{1,side}));
    static_rfoot_standing {side}  = filtfilt(b,a,(staticFoot_Orientation{1,side}));
    
    walking_pelvis_standing = filtfilt(b,a,(walkingPelvis_Orientation{1,1}));
    walking_rthigh_standing{side} = filtfilt(b,a,(walkingThigh_Orientation{1,side}));
    walking_rshank_standing {side} = filtfilt(b,a,(walkingShank_Orientation{1,side}));
    walking_rfoot_standing {side}  = filtfilt(b,a,(walkingFoot_Orientation{1,side}));
end

%% Calculating Joint Angles
d1 = static_pelvis_standing;
d2 = static_rthigh_standing;
d3 = static_rshank_standing;
d4 = static_rfoot_standing;

d5 = walking_pelvis_standing;
d6 = walking_rthigh_standing;
d7 = walking_rshank_standing;
d8 = walking_rfoot_standing;

for side=1:2
    
    [q_imu_pv_bf,q_imu_th_bf,q_imu_sh_bf,q_imu_ft_bf] = StandingCalib_IMU_MA(d1,d2{1,side},d3{1,side},d4{1,side},5,50);
    
    [Ahip_angle{side},Aknee_angle{side},Aankle_angle{side}]= JointAngles_IMU_MA(q_imu_pv_bf,q_imu_th_bf,q_imu_sh_bf,q_imu_ft_bf,d5,d6{1,side},d7{1,side},d8{1,side},1);
    
    %clearvars -except d1 d2 d3 d4 d5 d6 d7 d8 Ahip_angle Aknee_angle Aankle_angle
end

%% output data as csv file

M = [Aankle_angle{1}, Aankle_angle{2}, Ahip_angle{1}, Ahip_angle{2}, Aknee_angle{1}, Aknee_angle{2}];
filename = sprintf('jointangles_%s.csv', datestr(now, 'ddmmyyyy_HHMMSS'));
csvwrite(filename,M);

%% Plotting for testing purposes
plot(Aknee_angle{1,2}(:,1));hold on;plot(Aknee_angle{1,1}(:,1),'r')