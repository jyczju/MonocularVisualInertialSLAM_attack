% 初始化预积分变量
deltaP = zeros(3, 1); % 位移预积分
deltaV = zeros(3, 1); % 速度预积分
deltaR = eye(3);      % 旋转预积分

% 偏置初始化
bias_gyro = [0; 0; 0]; % 陀螺仪偏置
bias_accel = [0; 0; 0]; % 加速度计偏置

% 构造IMU measurements
imuMeasurements = struct('dt', [], 'gyro', [], 'accel', []);
imuMeasurements(1).dt = 0.01; % 时间间隔
imuMeasurements(1).gyro = [0.01; 0.02; 0.03]; % 陀螺仪测量值
imuMeasurements(1).accel = [0.1; 0.2; 0.3]; % 加速度计测量值
imuMeasurements(2).dt = 0.01; % 时间间隔
imuMeasurements(2).gyro = [0.02; 0.03; 0.04]; % 陀螺仪测量值
imuMeasurements(2).accel = [0.2; 0.3; 0.4]; % 加速度计测量值
imuMeasurements(3).dt = 0.01; % 时间间隔
imuMeasurements(3).gyro = [0.03; 0.04; 0.05]; % 陀螺仪测量值
imuMeasurements(3).accel = [0.3; 0.4; 0.5]; % 加速度计测量值
imuMeasurements(4).dt = 0.01; % 时间间隔
imuMeasurements(4).gyro = [0.04; 0.05; 0.06]; % 陀螺仪测量值
imuMeasurements(4).accel = [0.4; 0.5; 0.6]; % 加速度计测量值


% 遍历IMU测量值
for i = 1:length(imuMeasurements)
    dt = imuMeasurements(i).dt; % 时间间隔
    gyro = imuMeasurements(i).gyro - bias_gyro; % 去偏置角速度
    accel = imuMeasurements(i).accel - bias_accel; % 去偏置加速度

    % attack
    gyro = gyro + 0.1;

    % 更新旋转矩阵
    deltaR = deltaR * expm(skew(gyro * dt));

    % 更新速度和位移
    deltaV = deltaV + deltaR * accel * dt;
    deltaP = deltaP + deltaV * dt + 0.5 * deltaR * accel * dt^2;
end

% 输出预积分结果
disp('预积分位移 Δp:');
disp(deltaP);
disp('预积分速度 Δv:');
disp(deltaV);
disp('预积分旋转 ΔR:');
disp(deltaR);

% 辅助函数：计算反对称矩阵
function S = skew(omega)
    S = [  0,      -omega(3),  omega(2);
          omega(3),  0,      -omega(1);
         -omega(2), omega(1),  0      ];
end