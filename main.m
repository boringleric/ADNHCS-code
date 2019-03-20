clc;
A = load('berlin52.txt'); % 数据集
pop = 20;    % 种群数
gem = 500;   % 迭代次数
Pa = 0.2;    % 鸟巢发现概率 
[bestck, best_lenck]=finalver(A,pop,Pa,gem);     % 单次调用

% 循环30次取平均
a = 0;
for i=1:5
      [bestckr, best_lenckr]=finalver(A,pop,Pa,gem);
      a(i)=best_lenckr;
end
besta = min(a);
avra = mean(a,2);
besta
avra
