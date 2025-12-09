% 模拟攻击，运行这个脚本即可修改IMU的数据

clear
clc


fs = 100;
ts = 1/fs;
T = 3; % 攻击信号周期
t = 0:ts:T;
fb = 5; % 攻击频率Hz
c = 0.2; % 起始幅值
k = c/T; % 下降斜率
target_length = 25000; % 目标替换数组长度

A = c - k * t;
gyo_att_unit = A .* sin(2.*pi.*fb.*t);
acc_att_unit = - k * sin(2*pi.*fb.*t) + 2.*fb.*pi.*cos(2.*pi.*fb.*t).*(c - k*t);

gyo_att_unit = gyo_att_unit';
acc_att_unit = acc_att_unit';


max(abs(acc_att_unit))


% 加速度一般不超过10m/s2
if max(abs(acc_att_unit)) > 10
    acc_att_unit = acc_att_unit./max(abs(acc_att_unit)).*10*(c/2+0.5);
end

gyo_attack = signal_fill(gyo_att_unit,target_length);
acc_attack = signal_fill(acc_att_unit,target_length);


figure(1)
subplot(2,1,1)
plot(gyo_attack)
title("gyo attack signal")
subplot(2,1,2)
plot(acc_attack)
title("acc attack signal")



load('./BlackbirdVIOData/orig_data.mat')
figure(2)
subplot(2,1,1)
plot(gyroReadings(1:target_length, 2:3))
title("orig gyro signal")
subplot(2,1,2)
plot(accelReadings(1:target_length, 2:3))
title("orig acc signal")


gyroReadings(1:target_length, 2:3) = gyroReadings(1:target_length, 2:3) + repmat(gyo_attack,1,2);
accelReadings(1:target_length, 2:3)= accelReadings(1:target_length, 2:3) + repmat(acc_attack,1,2);


figure()
subplot(2,1,1)
plot(gyroReadings(1:target_length, 2:3))
title("attack gyro signal")
subplot(2,1,2)
plot(accelReadings(1:target_length, 2:3))
title("attack acc signal")


save('./BlackbirdVIOData/att_data_f5c02.mat')


function attack_signal = signal_fill(att_unit,target_length)
% 初始化目标数组
attack_signal = zeros(target_length,1);

% 循环填充目标数组
index = 1;
while index <= target_length - length(att_unit)
    % 确定当前循环填充的结束索引
    end_index = min(index + length(att_unit) - 1, target_length);
    
    % 将小数组放置到目标数组中
    attack_signal(index:end_index) = att_unit(1:end_index - index + 1);
    
    % 更新索引位置
    index = end_index + 1;
end
end