function [isAnomaly, combined_metric] = helperDetectAnomalousIMU(imuMeasurements, varargin)
% helperDetectAnomalousIMU 检测IMU数据是否异常
%
% 输入参数:
%   imuMeasurements - 包含陀螺仪和加速度计数据的结构体
%   varargin - 可选参数，包括阈值等
%
% 输出参数:
%   isAnomaly - 布尔值，指示是否检测到异常

% 解析可选参数
p = inputParser;
threshold = 0.5; % 默认阈值
addParameter(p, 'Threshold', threshold, @(x) isnumeric(x) && x > 0);
parse(p, varargin{:});

threshold = p.Results.Threshold;

% 计算陀螺仪和加速度计数据的方差和均值
gyro_variance = var(imuMeasurements.gyro, 0, 1);  % 沿着行计算每列的方差
accel_variance = var(imuMeasurements.accel, 0, 1); % 沿着行计算每列的方差

gyro_mean = mean(abs(imuMeasurements.gyro), 1);    % 计算绝对值的均值
accel_mean = mean(abs(imuMeasurements.accel), 1);  % 计算绝对值的均值

% 综合评估指标：高方差可能表示异常行为
combined_metric = sum(gyro_variance) + sum(accel_variance) + ...
                 sum(gyro_mean) + sum(accel_mean);

% 判断是否超过阈值
isAnomaly = combined_metric > threshold;

end