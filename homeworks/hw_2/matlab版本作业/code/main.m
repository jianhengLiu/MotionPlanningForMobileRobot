% Used for Motion Planning for Mobile Robots
% Thanks to HKUST ELEC 5660 
close all; clear all; clc;

% Renderer(扫描器)是一种图像数据转换程序，通过这个程序，可以把图像数据转化为Matlab可以用来画图的内容。
% 可以这样理解：Renderer(渲染器，描绘器)是matlab用来画图（即将数据转化为图像）的方式。
% 它三种方式：1）Painters   2)  Z-buffer  3)  OpenGL
% 
% 2.三种方式的比较
% 1)painters  再画简单的图画时，比较快；
% 2)z-buffer 又称深度缓冲技术，它是现在内存中画图，然后再将最终的图像显示在显示终端。这样的话，当在一个像素点，如果被多次赋值的话，它只会显示最高层的图像。
% 3) opengl 是利用硬件进行绘图的方式。
set(gcf, 'Renderer', 'painters');
% 设置图像的大小，分别代表x轴长度，y轴长度，图像长度，图像高度。
set(gcf, 'Position', [500, 50, 700, 700]);

% Environment map in 2D space 
xStart = 1.0;
yStart = 1.0;
xTarget = 9.0;
yTarget = 9.0;
MAX_X = 10;
MAX_Y = 10;
map = obstacle_map(xStart, yStart, xTarget, yTarget, MAX_X, MAX_Y);

% Waypoint Generator Using the A* 
[path,OPEN,CLOSED] = A_star_search(map, MAX_X,MAX_Y);
% visualize the 2D grid map
visualize_map(map, path,OPEN,CLOSED);
