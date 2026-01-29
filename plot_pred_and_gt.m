clear
clc


% 定义文件列表
file_list = {'trajectory_pred_gt_orig.mat','trajectory_pred_gt_attack.mat', 'trajectory_pred_gt_continue30.mat'};
legend_list = {'未攻击','攻击(持续240s)','攻击(持续30s)'};

% 创建一个新的图形窗口
figure;
h = gca; % 获取当前坐标轴的句柄
h.FontSize = 14; % 设置坐标轴上所有文字的字号为14，你可以根据需要调整这个值

% 循环遍历每一个文件
for i = 1:length(file_list)
    % 加载当前文件的数据
    load(file_list{i});
    
    % 对时间进行处理
    alignedTimes = alignedTimes - alignedTimes(1);
    alignedTimes = alignedTimes / 1e9 * 6.67;
    
    % 计算距离误差
    disError = sqrt(sum((pred - gt).^2, 2));
    
    % 绘制距离误差曲线
    % plot(alignedTimes, disError, 'LineWidth', 1.2, 'DisplayName', legend_list{i}); % 使用DisplayName属性为每条曲线添加图例
    semilogy(alignedTimes, disError, 'LineWidth', 1.2, 'DisplayName', legend_list{i}); % 使用DisplayName属性为每条曲线添加图例
    xlim([0,240])
    xticks([0:30:240])
    % ylim([0,1.2])
    % yticks([0, 0.2, 0.4, 0.6, 0.8,1.0,1.2])
    ylim([0,2.0])
    yticks([0,0.002, 0.005,0.01,0.02, 0.05,0.1,0.2,0.5,1.0,2.0])
    hold on; % 保持当前图像使得下一次plot命令在这个图上继续绘图
end

xlabel('时间(s)');
ylabel('偏差距离(m)');
grid on;

% 添加图例以区分不同的曲线
legend show;

% 完成绘图后释放hold
hold off;

% % === 新增：绘图 ===
% figure;
% error = abs(pred - gt);
% plot(alignedTimes, error(:,1), 'r', ...
%      alignedTimes, error(:,2), 'g', ...
%      alignedTimes, error(:,3), 'b', 'LineWidth', 1.2);
% xlabel('Time (s)');
% ylabel('Squared Error (m)');
% title('Squared Error (pred - gt)^2 vs Time');
% legend('X', 'Y', 'Z');
% grid on;


