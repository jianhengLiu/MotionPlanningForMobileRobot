function [path,OPEN,CLOSED] = A_star_search(map,MAX_X,MAX_Y)
%%
%This part is about map/obstacle/and other settings
%pre-process the grid map, add offset
% ��ȡ��ͼ���ϰ���+��ʼ��Ŀ��㣩�ĸ���
size_map = size(map,1);
% X��Y������ʼΪ0
Y_offset = 0;
X_offset = 0;

%Define the 2D grid map array.
%Obstacle=-1, Target = 0, Start=1
% ʹ��ͨ�п�Passable=2
MAP=2*(ones(MAX_X,MAX_Y));

%Initialize MAP with location of the target
% floor����˼�壬���ǵذ壬������ȡ����С�������������������ȡ��
% ԭ��Ϊ�˻�ͼ+0.5ʹ��λ�ڷ�������
% ��ȡĿ��㣬����Ӳ�����size_mapΪĿ�����±�)
xval=floor(map(size_map, 1)) + X_offset;
yval=floor(map(size_map, 2)) + Y_offset;
xTarget=xval;
yTarget=yval;
% ���Ŀ��㣬����Target=0
MAP(xval,yval)=0;

%Initialize MAP with location of the obstacle
% ����ϰ������Obstacle=-1
for i = 2: size_map-1
    xval=floor(map(i, 1)) + X_offset;
    yval=floor(map(i, 2)) + Y_offset;
    MAP(xval,yval)=-1;
end

%Initialize MAP with location of the start point
% �����ʼ�㣬����Start=1
xval=floor(map(1, 1)) + X_offset;
yval=floor(map(1, 2)) + Y_offset;
xStart=xval;
yStart=yval;
MAP(xval,yval)=1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%LISTS USED FOR ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%OPEN LIST STRUCTURE
%--------------------------------------------------------------------------
%IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
%--------------------------------------------------------------------------
OPEN=[];
%CLOSED LIST STRUCTURE
%--------------
%X val | Y val |
%--------------
% CLOSED=zeros(MAX_VAL,2);
CLOSED=[];

%Put all obstacles on the Closed list
%     ��Ϊ�ϰ��ﶼʱ�Ѿ������Ź���
k=1;%Dummy counter
for i=1:MAX_X
    for j=1:MAX_Y
        if(MAP(i,j) == -1)
            CLOSED(k,1)=i;
            CLOSED(k,2)=j;
            k=k+1;
        end
    end
end
CLOSED_COUNT=size(CLOSED,1);
%set the starting node as the first node
xNode=xval;
yNode=yval;
OPEN_COUNT=1;
goal_distance=distance(xNode,yNode,xTarget,yTarget);
path_cost=0;
OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,goal_distance,path_cost,goal_distance);
OPEN(OPEN_COUNT,1)=0;
CLOSED_COUNT=CLOSED_COUNT+1;
CLOSED(CLOSED_COUNT,1)=xNode;
CLOSED(CLOSED_COUNT,2)=yNode;
% �Ƿ��ҵ���·��
NoPath=1;

%%
%This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM

% ��ʼ������ʽ����h
h = ones(MAX_X,MAX_Y);
for i=1:MAX_X
    for j=1:MAX_Y
        if(MAP(i,j) == 2)
            h(i,j) = distance(xNode,yNode,xTarget,yTarget);
        end
    end
end
% ��ʼ�����ۺ���g
infinite = (MAX_X+MAX_Y);
g = infinite*ones(MAX_X,MAX_Y);
g(xStart,yStart) = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while(NoPath) %���յ㱻��ս���˳�ѭ��
    node_parent = CLOSED(CLOSED_COUNT,:);
    x_parent = node_parent(1,1);
    y_parent = node_parent(1,2);
    g_parent = g(x_parent,y_parent);
    
    for x = -1:1
        for y = -1:1
            x_expanding = x_parent+x;
            y_expanding = y_parent+y;
            if x_expanding>0 && x_expanding<=MAX_X && y_expanding>0 && y_expanding<=MAX_X && x+y~=0
                node_expanding = [x_expanding,y_expanding];
                % �ж���չ�ĵ��Ƿ���clost_list������չ���ϰ�����
                is_close = 0;
                for i = 1:CLOSED_COUNT
                    if CLOSED(i,:) == node_expanding
                        is_close = 1;
                        break;
                    end
                end
                if is_close ==1
                    continue;
                end
                
                hn = h(x_expanding,y_expanding);
                gn = g(x_expanding,y_expanding);
                costn = sqrt(x^2+y^2);
                if gn == infinite
                    gn = g_parent + costn;
                    g(x_expanding,y_expanding) = gn;
                    fn = gn + hn;
                    % ��δ��չ�Ľڵ����open_list��
                    OPEN_COUNT=OPEN_COUNT+1;
                    OPEN(OPEN_COUNT,:)=insert_open(x_expanding,y_expanding,x_parent,y_parent,hn,gn,fn);
                elseif gn > g_parent + costn
                    gn = g_parent + costn;
                    g(x_expanding,y_expanding) = gn;
                    fn = gn + hn;
                    for i = 1:OPEN_COUNT
                        if [OPEN(i,2),OPEN(i,3)] == node_expanding
                            OPEN(i,4) = x_parent;
                            OPEN(i,5) = y_parent;
                            OPEN(i,6) = gn;
                            OPEN(i,7) = fn;
                            break;
                        end
                    end
                end
            end
        end
    end
    
    Lowest_fn = 2*infinite;
    Lowest_fn_COUNT = 0;
    %     OPEN_COUNT
    for i = 1:OPEN_COUNT
        %         i
        if OPEN(i,1)==1
            if OPEN(i,end) < Lowest_fn
                Lowest_fn = OPEN(i,end);
                Lowest_fn_COUNT = i;
            end
        end
    end
    %     Lowest_fn_COUNT;
    %     OPEN;
    if Lowest_fn_COUNT == 0
        break
    end
        
    OPEN(Lowest_fn_COUNT,1)=0;
    x_closed = OPEN(Lowest_fn_COUNT,2);
    y_closed = OPEN(Lowest_fn_COUNT,3);
    CLOSED_COUNT=CLOSED_COUNT+1;
    CLOSED(CLOSED_COUNT,1)=x_closed;
    CLOSED(CLOSED_COUNT,2)=y_closed;
    
     % �Ƿ��ҵ���·��
    if CLOSED(CLOSED_COUNT,:) == [xTarget,yTarget]
        NoPath=0;
    end
    %
    %finish the while loop
    %
    
end %End of While Loop

%Once algorithm has run The optimal path is generated by starting of at the
%last node(if it is the target node) and then identifying its parent node
%until it reaches the start node.This is the optimal path

%
%How to get the optimal path after A_star search?
%please finish itOPEN
%
node_search = CLOSED(CLOSED_COUNT,:);
path = [];
k = 0;
while(~NoPath)
    k = k + 1;
    for i = 1:OPEN_COUNT
        if OPEN(i,1) == 0
            if [OPEN(i,2),OPEN(i,3)] == node_search
                path(k,:) = node_search;
                node_search = [OPEN(i,4),OPEN(i,5)];
                break;
            end
        end
    end
    if node_search == [xStart,yStart]
        break
    end
end
path(k+1,:) = node_search;
end
