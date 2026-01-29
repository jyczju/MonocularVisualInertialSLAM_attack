clear
clc

% Load data
% accelReadings, gyroReadings, images, gTruth, timeStamps
load('./BlackbirdVIOData/orig_data.mat')
%% 帧号
accel_fix_frame  = 10801;
gyro_fix_frame   = 10801;
image_fix_frame  = 400;
gTruth_fix_frame = 2174;

%% 加速度计（double）
Na = size(accelReadings,1);
accelReadings(accel_fix_frame:end,:) = ...
    repmat([0 0 -9.78], Na - accel_fix_frame + 1, 1);

%% 陀螺仪（double）
Ng = size(gyroReadings,1);
gyroReadings(gyro_fix_frame:end,:) = ...
    repmat([0 0 0], Ng - gyro_fix_frame + 1, 1);

%% 图像（cell）
for i = image_fix_frame+1:numel(images)
    images{i} = images{image_fix_frame};
end

%% gTruth（table）
Nt = height(gTruth);
for i = gTruth_fix_frame+1:Nt
    gTruth(i, :) = gTruth(gTruth_fix_frame, :);
end



% Save the modified data back if necessary
save('./BlackbirdVIOData/data_stable.mat')
