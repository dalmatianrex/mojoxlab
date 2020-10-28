clear all;
staticFile = '/Users/riasatislam/Documents/OU_PhD/cardiff backup/data/Subj2/Session1/Exp/Trial-002#My MVN System.mvnx';
walkingFile = '/Users/riasatislam/Documents/OU_PhD/cardiff backup/data/Subj2/Session1/Exp/Trial-008#My MVN System.mvnx';

staticTree = load_mvnx(staticFile);
walkingTree = load_mvnx(walkingFile);

l = length(staticTree.subject.frames.frame);
for i = 3:l
    staticQuaternions(i,1:4) = staticTree.subject.frames.frame(i).sensorOrientation(1:4); % pelvis
    staticQuaternions(i,5:8) = staticTree.subject.frames.frame(i).sensorOrientation(5:8); % right thigh
    staticQuaternions(i,9:12) = staticTree.subject.frames.frame(i).sensorOrientation(9:12); % right shank
    staticQuaternions(i,13:16) = staticTree.subject.frames.frame(i).sensorOrientation(13:16); % right foot
    staticQuaternions(i,17:20) = staticTree.subject.frames.frame(i).sensorOrientation(17:20); % left thigh
    staticQuaternions(i,21:24) = staticTree.subject.frames.frame(i).sensorOrientation(21:24); % left shank
    staticQuaternions(i,25:28) = staticTree.subject.frames.frame(i).sensorOrientation(25:28); % left foot
end

staticQuaternions = staticQuaternions(3:end,1:end);

l = length(walkingTree.subject.frames.frame);
for i = 3:l
    walkingQuaternions(i,1:4) = walkingTree.subject.frames.frame(i).sensorOrientation(1:4); % pelvis
    walkingQuaternions(i,5:8) = walkingTree.subject.frames.frame(i).sensorOrientation(5:8); % right thigh
    walkingQuaternions(i,9:12) = walkingTree.subject.frames.frame(i).sensorOrientation(9:12); % right shank
    walkingQuaternions(i,13:16) = walkingTree.subject.frames.frame(i).sensorOrientation(13:16); % right foot
    walkingQuaternions(i,17:20) = walkingTree.subject.frames.frame(i).sensorOrientation(17:20); % left thigh
    walkingQuaternions(i,21:24) = walkingTree.subject.frames.frame(i).sensorOrientation(21:24); % left shank
    walkingQuaternions(i,25:28) = walkingTree.subject.frames.frame(i).sensorOrientation(25:28); % left foot
end

walkingQuaternions = walkingQuaternions(3:end,1:end);

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