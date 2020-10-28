MoJoXlab is a MATLAB based custom motion capture analysis software toolkit whose aim is to produce freely available motion capture analysis software to be used by anyone interested in generating lower limb joint kinematics waveforms using any suitable wearable inertial measurement units (IMUs).

It can generate joint angles of Ankle, Hip and Knee joints in Sagittal, Frontal and Transverse (not available for Ankle joint) planes for different functional tasks such as walking, squatting, and jumping. The joint kinematics for MoJoXlab is based on Joint Coordinate System (JCS) as proposed by Grood and Suntay. MoJoXlab takes sensor orientation data in quaternions as input and outputs joint angles in degrees. The algorithm considers a static calibration step, where sensor data is captured for calibration purposes while the participant maintains a standing pose. This calibration step is then used to calculate the joint angles for the dynamic phase of the motion.

Currently, MoJoXlab is designed to be used on a 7 sensor based setup, with sensors placed in the Pelvis, both thighs, shanks and feet. Input files should be in quaternions for 7 sensors with 28 columns of data in the following order, where each sensor consists of 4 columns of quaternion orientations:
Pelvis, Right Thigh, Right Shank, Right Foot, Left Thigh, Left Shank, Left Foot.

Output files will contain joint angles with 16 columns of data in the following order:
Ankle Left Sagittal, Ankle Left Frontal, Ankle Right Sagittal, Ankle Right Frontal,
Hip Left Sagittal, Hip Left Frontal, Hip Left Transverse, Hip Right Sagittal, Hip Right Frontal, Hip Right Transverse,
Knee Left Sagittal, Knee Left Frontal, Knee Left Transverse, Knee Right Sagittal, Knee Right Frontal, Knee Right Transverse.


Installation: -

Windows:
Install MoJoXlab using the setup.exe file available in the zip folder. Use the default settings for installation. After installation is complete, run the application from the installation directory.

MacOS:
Install MoJoXlab using the application installer file available in the zip folder. Use the default settings for installation. After installation is complete, run the MoJoXlab application from the Applications folder.

The software workflow: -

Get Static Calibration Data:
Use this button to select the static calibration data file. A dialog box will open to allow you to select the calibration data file. Be careful as sometimes the dialog box opens behind the app and becomes difficult to see. After you have selected the calibration data file. The filepath and filename is shown on the adjacent text field. 
[sample data is provided with the main zip folder with filename: sample_StaticCalibrationDataFile.csv]

Get Activity Data:
Use this button to select the dynamic activity file for example, walking, jumping or squatting activity data file. A dialog box will open to allow you to select the calibration data file. Be careful as sometimes the dialog box opens behind the app and becomes difficult to see. After you have selected the calibration data file. The filepath and filename is shown on the adjacent text field. 
[sample data is provided with the same zip folder with filename: sample_ActivityDataFile.csv]

Sampling Frequency:
Use this field to input the sampling frequency of the activity data in Hertz (Hz). [use 60 Hz for sample data]

Calculate Joint Angles:
Use this button to calculate the joint angles. A dialog box will open that will allow you to select the output folder. Be careful as sometimes the dialog box opens behind the app and becomes difficult to see. The output data file will be saved in this folder. The filename is automatically generated and will be shown on the adjacent text field.

Joint Angle Waveforms:
The plotting area shows the joint angles calculated in degrees, for either Ankle, Hip or Knee joints, showing left or right side, for either sagittal, frontal or transverse planes.

For Your Own Use:
Input files should be in quaternions for 7 sensors with 28 columns of data in the following order: 
Pelvis, Right Thigh, Right Shank, Right Foot, Left Thigh, Left Shank, Left Foot.

Output files will contain joint angles with 16 columns of data in the following order:
Ankle Left Sagittal, Ankle Left Frontal, Ankle Right Sagittal, Ankle Right Frontal, 
Hip Left Sagittal, Hip Left Frontal, Hip Left Transverse, Hip Right Sagittal, Hip Right Frontal, Hip Right Transverse,
Knee Left Sagittal, Knee Left Frontal, Knee Left Transverse, Knee Right Sagittal, Knee Right Frontal, Knee Right Transverse. 