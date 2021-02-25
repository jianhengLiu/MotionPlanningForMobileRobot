function visualize_map(map,path,OPEN,CLOSED)
%This function visualizes the 2D grid map
%consist of obstacles/start point/target point/optimal path.

% obstacles
% size(map, 1)返回地图(障碍物+起始和目标点)的个数，即返回map数组第一通道的数目
% 跳过开始点开始添加
for obs_cnt = 2: size(map, 1) - 1
    % 创建散点图，即生成点
    % scatter(x,y,点的大小，颜色，'填充')
    % -0.5是为了保证添加的障碍点在方格中央
    scatter(map(obs_cnt, 1)-0.5,map(obs_cnt, 2)-0.5,250,155,'filled');
    % 保持图像和坐标格网格
    hold on;
    grid on;
    %grid minor;
    % axis equal 将横轴纵轴的定标系数设成相同值 ,即单位长度相同
    axis equal;
    % 设定横纵坐标范围
    axis ([0 10 0 10 ]);
    hold on;
end
% start point
% 起始点标记为蓝色星号
scatter(map(1, 1)-0.5, map(1, 2)-0.5,'b','*');
hold on;
% target point
% 终点标记为红色星号
scatter(map(size(map, 1), 1)-0.5, map(size(map, 1), 2)-0.5, 'r','*');
hold on;
%optimal path
% 估计or规划的路径
for path_cnt = 2:size(path,1)-1
    scatter(path(path_cnt,1)-0.5,path(path_cnt,2)-0.5,'b');
    hold on;
end

%     搜索过的OPEN区块
%     for OPEN_cnt = 1:size(OPEN,1)
%         scatter(OPEN(OPEN_cnt,2)-0.5,OPEN(OPEN_cnt,3)-0.5,300,'r','o');
%         hold on;
%     end

%     搜索过的CLOSED区块
    for CLOSED_cnt = 1:size(CLOSED,1)
        scatter(CLOSED(CLOSED_cnt,1)-0.5,CLOSED(CLOSED_cnt,2)-0.5,300,'r','o');
        hold on;
    end

end

