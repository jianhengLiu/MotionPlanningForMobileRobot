 function map = obstacle_map(xStart,yStart,xTarget,yTarget,MAX_X,MAX_Y)
%This function returns a map contains random distribution obstacles.
    %获取0-1均匀分布的随机数组[MAX_X,MAX_Y]
    rand_map = rand(MAX_X,MAX_Y);
    map = [];
    map(1,1) = xStart;
    map(1,2) = yStart;
    k=2;
    %障碍概率
    obstacle_ratio = 0.25;
    for i = 1:1:MAX_X
        for j = 1:1:MAX_Y
            %判断是否要添加障碍物，需要添加则赋予坐标值
            %matlab中~=为不等于的意思，即C中的!=
            if( (rand_map(i,j) < obstacle_ratio) && (i~= xStart || j~=yStart) && (i~= xTarget || j~=yTarget))
                map(k,1) = i;
                map(k,2) = j;
                k=k+1;
            end    
        end
    end
    %在数组末尾添加目标坐标
    map(k,1) = xTarget;
    map(k,2) = yTarget;
end

