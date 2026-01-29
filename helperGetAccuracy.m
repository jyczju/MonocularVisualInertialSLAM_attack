function [rmse, ame] = helperGetAccuracy(viPoses,data,keyTimeStamps, gDir)

pred=[];
gt=[];
timeStamps=data.timeStamps;

gTruth = data.gTruth(:, 1:3);

gDirInv = se3(invert(gDir).A);
imuToCam = inv(data.camToIMUTransform);
gTruth = imuToCam.transform(gTruth);
gTruth = gDirInv.transform(gTruth);

% 提取预测和真值，并对齐时间戳
alignedTimes = [];  % 用于绘图的时间轴
for i=1:height(viPoses)
    poseVI=viPoses.AbsolutePose(i).Translation;
    pred=[pred;poseVI];
    viTimeStamp=keyTimeStamps(i);
    [~,idx] = min(abs(timeStamps.imageTimeStamps - viTimeStamp));
    poseGT=gTruth(idx,1:3);
    gt=[gt;poseGT];
    alignedTimes = [alignedTimes; timeStamps.imageTimeStamps(idx)];
end

rmse = sqrt(mean(sum((pred - gt).^2,2)));
ame = mean(sum(abs(pred - gt),2));



disp(["AME for trajectory (m): ",num2str(ame)])
disp(["RMSE for trajectory (m): ",num2str(rmse)])

% === 新增：保存 pred 和 gt 到 .mat 文件 ===
save('trajectory_pred_gt.mat', 'pred', 'gt', 'alignedTimes');


% === 新增：绘图 ===
figure;
squaredError = (pred - gt).^2;
plot(alignedTimes/1e9, squaredError(:,1), 'r', ...
     alignedTimes/1e9, squaredError(:,2), 'g', ...
     alignedTimes/1e9, squaredError(:,3), 'b', 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Squared Error (m^2)');
title('Squared Error (pred - gt)^2 vs Time');
legend('X', 'Y', 'Z');
grid on;


end