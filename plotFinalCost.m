clear
clc

% 自动加载并获取唯一变量
S1 = load('finalCostList.mat');
S2 = load('finalCostList_attack_best.mat');
S3 = load('finalCostList_attack_defense.mat');

% 获取结构体中的第一个字段（即变量名）
var1 = fieldnames(S1);
var2 = fieldnames(S2);
var3 = fieldnames(S3);

normal = S1.(var1{1})/2927; % 除以点数量
attack = S2.(var2{1})/2927;
defense = S3.(var3{1})/2927;


% 绘图（同上）
figure;
% plot(normal(63:63+50), 'b', 'DisplayName', '无攻击', 'LineWidth', 0.9);
% hold on;
% plot(attack(63:63+50), 'r', 'DisplayName', '有攻击', 'LineWidth', 0.9);
% hold off;

plot(normal(170:220), 'b', 'DisplayName', '无攻击', 'LineWidth', 0.9);
hold on;
plot(attack(170:220), 'r', 'DisplayName', '有攻击无防御', 'LineWidth', 0.9);
hold on;
plot(normal(170:220), 'g--', 'DisplayName', '有攻击有防御', 'LineWidth', 0.9);

% plot(normal, 'b', 'DisplayName', '无攻击', 'LineWidth', 0.9);
% hold on;
% plot(attack, 'r', 'DisplayName', '有攻击无防御', 'LineWidth', 0.9);
% hold on;
% plot(normal, 'g--', 'DisplayName', '有攻击有防御', 'LineWidth', 0.9);

hold off;


xlabel('帧数', 'FontName', 'SimHei');
ylabel('cost值', 'FontName', 'SimHei');
% title('Cost Comparison', 'FontName', 'SimHei');  % 如果启用标题也加字体
xlim([0,51])
legend('show', 'FontName', 'SimHei');

grid on;

% 设置坐标轴刻度标签字体（包括 x 和 y 轴数字）
ax = gca;
ax.FontName = 'SimHei';