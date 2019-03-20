function [best, bestsofar] = finalver(A,pop,Pa,gem)
figure('name','testTSP');
% 画出散点图
plot(A(:,2),A(:,3),'s','markersize',2);   

% 截取数据点坐标
X=A(:,2:3);  
Mdl = KDTreeSearcher(X);    % 筛选最近邻

%初始化参数定义部分
[N,~]=size(A);              % 获取城市数
D=distance(A);              % 生成距离矩阵
PP=zeros(pop,N+1);
P0=zeros(pop,N+1);
for i=1:pop
    P0(i,2:N)=randperm(N-1)+1;    % 随机生成初始群体P0
end

for i=1:pop
    PP(i,:)=[1 P0(i,2:N) 1];      % 修改形成初始群体PP,添加起点终点
end

% 构造改良圈算法初始矩阵
for k=1:pop
    flag=1;
    while flag
        flag=0;
        for i=1:N-2
            for j=i+2:N
                if D(PP(k,i),PP(k,j))+D(PP(k,i+1),PP(k,j+1))<D(PP(k,i),PP(k,i+1))+D(PP(k,j),PP(k,j+1))
                   PP(k,(i+1):j)=PP(k,j:-1:(i+1));
                   flag=1;
                 end
             end
         end
     end
end

P1 = PP(:, 1:N);               % 截取路程
fit_ret = fitness(P1, D);      % 计算适应度函数
[route_sr,best_sr] = sort(fit_ret);    % 适应度排序
best_len = route_sr(1);       % 当前最短路径总和
best = P1(best_sr(1), :);     % 截取子路程
bestsofar = best_len;
total = gem;
tabulength= 5+N; % 禁忌长度 % 
global tabulist;
% 初始化禁忌表
for i = 1:tabulength
   tabulist(i).list= 0;
   tabulist(i).value= 0;
end
% 开始迭代
while gem
    % 种群选择 parfor帮助提速
    parfor i=1:pop              
        B = P1(i,:);
        locbest = fitness(B, D);    % 计算适应度 
        bestch = B;                 % 暂存结果
        temp = rand;                
        if temp < (0.5-0.1*(1/(1+exp(-(gem-450)/10)))-0.2*(1/(1+exp(-(gem-250)/10))))                 %列维飞行，长短距离
            choice = 1;  % 2-opt邻域
        elseif temp > (0.9-0.3*(1/(1+exp(-(gem-450)/10))))     
            choice = 4;  % 双桥邻域
        else  
            choice = 2;  % 3-opt邻域
        end
        switch choice
            case 1
                [E] = crossover(Mdl,A,B,D,1,gem);           % 2-opt邻域
            case 2
                if gem > 350
                     [E] = crossover(Mdl,A,B,D,3,gem);      % 3-opt前期
                else
                     [E] = crossover(Mdl,A,B,D,2,gem);      % 3-opt后期
                end 
            case 4
                 [E] = crossover(Mdl,A,B,D,4,gem);          % 双桥邻域
        end
        C = E;
        Fitnc = fitness(C,D);      % 计算适应度 
        if Fitnc < locbest
            bestch = C;
            locbest = Fitnc;
        end 
       
        % 鸟巢被发现 
        if rand < Pa                  
               [F] =  crossover(Mdl,A,B,D,4,500);           % 双桥邻域
               G = F;
               Fitnc = fitness(G,D);                    % 计算适应度 
               if Fitnc < locbest
                  bestch = G;
               end 
        end
        P1(i,:) = bestch;
    end

    P = P1;
    fit_ret = fitness(P, D);              % 计算适应度函数
    [route_sr,best_sr] = sort(fit_ret);   % 适应度排序
    best_len = route_sr(1);               % 当前最短路径总和
    best = P(best_sr(1) ,:);              % 截取子路程
    % 更新最优解
    if bestsofar > best_len
        bestsofar = best_len;
    end
    Saver(1,total - gem+1) = bestsofar;    %存储每次的最优解
    gem = gem-1;     % 进入下一次循环
end

% 画出闭合路径曲线图
scatter(A(:,2),A(:,3),'x');
hold on;
plot([A(best(1),2),A(best(N),2)],[A(best(1),3),A(best(N),3)]);
hold on;
title(best_len)  %添加图像标题
for i=1:N-1
    x0=A(best(i),2);
    x1=A(best(i+1),2);
    y0=A(best(i),3);
    y1=A(best(i+1),3);
    xx=[x0, x1];
    yy=[y0, y1];
    plot(xx,yy);
    hold on;
end
figure(3), plot(1:length(Saver), Saver, 'r-'); legend('最优路径'); title(bestsofar); xlabel('迭代次数'); ylabel('路程距离')

% 计算距离矩阵
function D = distance(A)          
    [N,~]=size(A);
	D=zeros(N);
    for i=1:N
        for j=i:N
            D(i,j)=round(sqrt((A(i,2)-A(j,2)).^2+(A(i,3)-A(j,3)).^2));
            D(j,i)=D(i,j);
        end
    end
    
% 计算闭合路径值(矩阵）
function Fit = fitness(P,D)       
    pop=size(P,1);
    Fit=zeros(pop,1); 
    [N,~]=size(D);
    for i=1:pop
        for j=1:N-1
          Fit(i,1)=Fit(i,1)+D(P(i,j),P(i,j+1));
        end
        Fit(i,1)=Fit(i,1)+D(P(i,N),P(i,1));
    end
    
% 布谷鸟计算全部
function [B] = crossover(Mdl,A,B,D,choice,gem)      
    switch choice
        case 1              % 2-opt邻域
            for i=1:20
                tmpB = B(1,2:size(B,2));
                fitbest = fitness(B,D);
                len = size(tmpB,2);                 % 片段长度
                avelen = fitbest/len;
                avelen = avelen*2;
                getpoint = tmpB(1,randperm(len,1)); % 随机选点
                [X] = A(getpoint,2:3);
                Idx = rangesearch(Mdl,X,avelen);    % 计算最近邻
                as = cell2mat(Idx);
                chose = intersect(as,tmpB);         % 排除第一个
                if size(chose,2) == 1
                    point1 = find(tmpB == chose(1,1));
                    point2 = getpoint;
                else
                    getpoint = randperm(size(chose,2),2);
                    point1 = find(tmpB == chose(getpoint(1,1)));
                    point2 = find(tmpB == chose(getpoint(1,2)));
                end
                
                if point1 > point2
                    tmp = point1;
                    point1 = point2;
                    point2 = tmp;              
                end
                if point2 == point1+1 
                    if point2 == len
                        --point1;
                    else
                        ++point2;
                    end
                end
                a2 = point1;
                a1 = a2-1;
                b2 = point2;
                b1 = b2-1;
                
                sec1 = tmpB(1,1:a1);                % 取第一部分
                secfli1 = fliplr(sec1);             % 第一部分反序
                
                sec2 = tmpB(1,a2:b1);               % 取第二部分
                secfli2 = fliplr(sec2);             % 第二部分反序
                
                sec3 = tmpB(1,b2:size(tmpB,2));     % 取第三部分
                secfli3 = fliplr(sec3);             % 第三部分反序
                
                subans1 = [secfli1';sec2';sec3']';  % a'bc = ac'b'
                subans2 = [sec1';secfli2';sec3']';  % ab'c
                subans3 = [sec1';sec2';secfli3']';  % abc'
                
                AnsSub(3*(i-1)+1).seris = [1 subans1];
                AnsSub(3*(i-1)+1).changef = 1;
                AnsSub(3*(i-1)+1).changel = len+1;
                AnsSub(3*(i-1)+2).seris = [1 subans2];
                if a1 == 0
                    a1 = 1;
                end
                AnsSub(3*(i-1)+2).changef = a1;
                AnsSub(3*(i-1)+2).changel = len+1;
                AnsSub(3*(i-1)+3).seris = [1 subans3];
                AnsSub(3*(i-1)+3).changef = b1;
                AnsSub(3*(i-1)+3).changel = len+1;
            end
            [E] = returnbest(AnsSub,D,fitbest,1,gem);
            if E ~= 0
               B = E;
            end

       case 2            % 3opt(动态)
           for i=1:20
               tmpB=B(1,2:size(B,2));
               fitbest = fitness(B,D);
               len = size(tmpB,2);                  % 片段长度
               avelen = fitbest/len;
               avelen = avelen*3;
               getpoint = tmpB(1,randperm(len,1));
               [X] = A(getpoint,2:3);
               Idx = rangesearch(Mdl,X,avelen);
               as = cell2mat(Idx);
               chose = intersect(as,tmpB);          % 排除第一个

               if size(chose,2) <= 6
                   [vec] = 2:len-4;
                   getpoint = vec(randperm(length(vec),1));
                   a2 = getpoint(1);
                   a1 = a2-1;
                   sec1 = tmpB(1,1:a1);             % 取第一部分
                   secfli1 = fliplr(sec1);          % 第一部分反序
                   
                   [vec] = a2+2:len-2;
                   getpoint = vec(randperm(length(vec),1));
                   b2 = getpoint(1);
                   b1 = b2-1;
                   sec2 = tmpB(1,a2:b1);           % 取第二部分
                   secfli2 = fliplr(sec2);         % 第二部分反序
                   
                   [vec] = b2+2:len;
                   getpoint = vec(randperm(length(vec),1));
                   c2 = getpoint(1);
                   c1 = c2-1;
                   sec3 = tmpB(1,b2:c1);           % 取第三部分
                   secfli3 = fliplr(sec3);         % 第三部分反序
                   
                   sec4 = tmpB(1,c2:len);          % 取第四部分
                   secfli4 = fliplr(sec4);         % 第四部分反序
               else %候选集足够大
                  for cnt=1:size(chose,2)
                      candplc(1,cnt) = find(tmpB == chose(1,cnt));
                  end
                  candplcsort = sort(candplc);
                  lencand = size(candplcsort,2);
                  [vec] = 2:lencand-4;              % 下标候选截断位置
                  getpoint = randperm(length(vec),1); % 找下标
                  gpa = vec(1,getpoint);            % 对应原位置
                  a2 = candplcsort(1,gpa);          % 候选映射
                  a1 = a2-1;
                  sec1 = tmpB(1,1:a1);              % 取第一部分
                  secfli1 = fliplr(sec1);           % 第一部分反序
                  
                  [vec] = gpa+2:lencand-2;
                  getpoint = randperm(length(vec),1);
                  gpb = vec(1,getpoint);
                  b2 = candplcsort(1,gpb);
                  b1 = b2-1;
                  sec2 = tmpB(1,a2:b1);             % 取第二部分
                  secfli2 = fliplr(sec2);           % 第二部分反序
                  
                  [vec] = gpb+2:lencand;
                  getpoint = randperm(length(vec),1);
                  gpc = vec(1,getpoint);
                  c2 = candplcsort(1,gpc);
                  c1 = c2-1;
                  sec3 = tmpB(1,b2:c1);             % 取第三部分
                  secfli3 = fliplr(sec3);           % 第三部分反序
                  
                  sec4 = tmpB(1,c2:len);            % 取第四部分
                  secfli4 = fliplr(sec4);           % 第四部分反序
               end
              subans1 = [sec1';secfli2';secfli3';sec4']';           %ab'c' = a'cb
              subans2 = [secfli1';secfli2';sec3';secfli4']';        %a'b'c
              subans3 = [secfli1';sec2';secfli3';secfli4']';        %a'bc'
              subans4 = [secfli1';secfli2';secfli3';secfli4']';     %a'b'c'
              
               AnsSub(4*(i-1)+1).seris = [1 subans1];
                if a1 == 0
                    a1 = 1;
                end
                AnsSub(4*(i-1)+1).changef = a1;
                AnsSub(4*(i-1)+1).changel = len+1;
                AnsSub(4*(i-1)+2).seris = [1 subans2];
                AnsSub(4*(i-1)+2).changef = 1;
                AnsSub(4*(i-1)+2).changel = len+1;
                AnsSub(4*(i-1)+3).seris = [1 subans3];
                AnsSub(4*(i-1)+3).changef = 1;
                AnsSub(4*(i-1)+3).changel = len+1;
                AnsSub(4*(i-1)+4).seris = [1 subans4];
                AnsSub(4*(i-1)+4).changef = 1;
                AnsSub(4*(i-1)+4).changel = len+1;
          end
              [E] = returnbest(AnsSub,D,fitbest,1,gem);
              if E ~= 0
                  B = E;
              end
  
        case 3            %3opt(无动态)
            for i=1:20
                tmpB=B(1,2:size(B,2));
                fitbest = fitness(B,D);
                len = length(tmpB);         %片段长度
                
                [vec] = 2:len-4;
                getpoint = vec(randperm(length(vec),1));
                a2 = getpoint(1);
                a1 = a2-1;
                sec1 = tmpB(1,1:a1);         %取第一部分
                secfli1 = fliplr(sec1);     %第一部分反序
                
                [vec] = a2+2:len-2;
                getpoint = vec(randperm(length(vec),1));
                b2 = getpoint(1);
                b1 = b2-1;
                sec2 = tmpB(1,a2:b1);        %取第二部分
                secfli2 = fliplr(sec2);     %第二部分反序
                
                [vec] = b2+2:len;
                getpoint = vec(randperm(length(vec),1));
                c2 = getpoint(1);
                c1 = c2-1;
                sec3 = tmpB(1,b2:c1);        %取第二部分
                secfli3 = fliplr(sec3);     %第二部分反序
                
                sec4 = tmpB(1,c2:len);       %取第三部分
                secfli4 = fliplr(sec4);     %第三部分反序
                
                subans1 = [sec1';secfli2';secfli3';sec4']';       %ab'c' = a'cb
                subans2 = [secfli1';secfli2';sec3';secfli4']';       %a'b'c
                subans3 = [secfli1';sec2';secfli3';secfli4']';       %a'bc'
                subans4 = [secfli1';secfli2';secfli3';secfli4']';       %a'b'c'
                
                AnsSub(4*(i-1)+1).seris = [1 subans1];
                AnsSub(4*(i-1)+1).changef = a1;
                AnsSub(4*(i-1)+1).changel = len+1;
                AnsSub(4*(i-1)+2).seris = [1 subans2];
                AnsSub(4*(i-1)+2).changef = 1;
                AnsSub(4*(i-1)+2).changel = len+1;
                AnsSub(4*(i-1)+3).seris = [1 subans3];
                AnsSub(4*(i-1)+3).changef = 1;
                AnsSub(4*(i-1)+3).changel = len+1;
                AnsSub(4*(i-1)+4).seris = [1 subans4];
                AnsSub(4*(i-1)+4).changef = 1;
                AnsSub(4*(i-1)+4).changel = len+1;
            end
            [E] = returnbest(AnsSub,D,fitbest,1,gem);
            if E ~= 0
                B = E;
            end
            
        case 4            %双桥
            for i=1:20
                tmpB=B(1,2:size(B,2));
                fitbest = fitness(B,D);
                len = length(tmpB);         %片段长度
                
                [vec] = 2:len-6;
                getpoint = vec(randperm(length(vec),1));
                a2 = getpoint(1);
                a1 = a2-1;
                sec1 = tmpB(1,1:a1);         %取第一部分
                secfli1 = fliplr(sec1);     %第一部分反序
                
                [vec] = a2+2:len-4;
                getpoint = vec(randperm(length(vec),1));
                b2 = getpoint(1);
                b1 = b2-1;
                [vec] = b2+2:len-2;
                sec2 = tmpB(1,a2:b1);        %取第二部分
                secfli2 = fliplr(sec2);     %第二部分反序
                
                getpoint = vec(randperm(length(vec),1));
                c2 = getpoint(1);
                c1 = c2-1;
                [vec] = c2+2:len;
                sec3 = tmpB(1,b2:c1);       %取第三部分
                secfli3 = fliplr(sec3);     %第三部分反序
                
                getpoint = vec(randperm(length(vec),1));
                d2 = getpoint(1);
                d1 = d2-1;
                sec4 = tmpB(1,c2:d1);       %取第四部分
                secfli4 = fliplr(sec4);     %第四部分反序
                
                sec5 = tmpB(1,d2:len);      %取第五部分
                secfli5 = fliplr(sec5);     %第五部分反序
                
                subans1 = [sec1';sec4';sec3';sec2';sec5']';       %双桥
                subans2 = [secfli1';sec4';secfli3';sec2';secfli5']';
                subans3 = [secfli1';sec2';secfli3';sec4';secfli5']';
                subans4 = [sec1';secfli4';sec3';secfli2';sec5']';
                subans5 = [sec1';secfli2';sec3';secfli4';sec5']';
                
                AnsSub(5*(i-1)+1).seris = [1 subans1];
                AnsSub(5*(i-1)+1).changef = a1;
                AnsSub(5*(i-1)+1).changel = len+1;
                AnsSub(5*(i-1)+2).seris = [1 subans2];
                AnsSub(5*(i-1)+2).changef = 1;
                AnsSub(5*(i-1)+2).changel = len+1;
                AnsSub(5*(i-1)+3).seris = [1 subans3];
                AnsSub(5*(i-1)+3).changef = 1;
                AnsSub(5*(i-1)+3).changel = len+1;
                AnsSub(5*(i-1)+4).seris = [1 subans4];
                AnsSub(5*(i-1)+4).changef = a1;
                AnsSub(5*(i-1)+4).changel = len+1;
                AnsSub(5*(i-1)+5).seris = [1 subans5];
                AnsSub(5*(i-1)+5).changef = a1;
                AnsSub(5*(i-1)+5).changel = len+1;
            end
            [E] = returnbest(AnsSub,D,fitbest,1,gem);
            if E ~= 0
                B = E;
            end
    end
 
% 计算最优解
function [E] = returnbest(C,D,fitbest,opt,gem)
    global tabulist;
    E = [];
    arraylength = size(C,2);
    tabulength = size(tabulist,2);
    for i=1:arraylength                     % 2 opt优化
        genbe = C(i).seris;      
        if opt == 1
            totalen = size(genbe,2);
            startpos = C(i).changef;
            endpos = C(i).changel;
            if startpos + 3 > totalen
                startpos = startpos - 3;
            end
            flag = 1;
            if rand < 0.2 + 0.3 * (1/(1+exp(-(gem-400)/10)))    %dropout
                flag = 0;
            end
            while flag
                flag=0;
                for x=startpos:endpos-3
                    for j=x+2:endpos-1
                            if D(genbe(1,x),genbe(1,j))+D(genbe(1,x+1),genbe(1,j+1))<D(genbe(1,x),genbe(1,x+1))+D(genbe(1,j),genbe(1,j+1))
                                genbe(1,(x+1):j)=genbe(1,j:-1:(x+1));
                                flag=1;
                        end
                    end
                end
            end
        end
        neighborhood(i).list = genbe;     % 候选集的城市序列
        fitnessc = fitness(genbe,D);
        neighborhood(i).value= fitnessc;
    end
    candidate_next= sort(cat(1, neighborhood.value), 1).'; % 将候选集的目标值按从小到大排序
    
    if candidate_next(1) < fitbest               %满足藐视准则
       index = find(cat(1, neighborhood.value)==candidate_next(1)); % 找到最小目标值对应的对象的位置
       L = length(index); % 有可能不只一个最小值
       if L > 1 % 如果不只一个最小值 选择后面一种
           candidate_now.list = neighborhood(index(L)).list;
       else % 只有一个最小值
           candidate_now.list = neighborhood(index(1)).list;
       end
       candidate_now.value = candidate_next(1);
        
       for i = tabulength-1:-1:1              % 更新禁忌表
           tabulist(i+1).list = tabulist(i).list;
           tabulist(i+1).value = tabulist(i).value;
       end
       tabulist(1).list = candidate_now.list;
       tabulist(1).value = candidate_now.value;
       E = tabulist(1).list;
    else
       for n = 1:size(candidate_next,1)
           if candidate_next(1) > ceil(fitbest*1.07)    % 早停
               return;
           end
           index = find(cat(1, neighborhood.value)==candidate_next(n));
           tx = neighborhood(index(1)).list;
           % 判断是否在禁忌表中
           flag = true;
           for i = 1:tabulength
               if tabulist(i).value == 0
                   continue;
               else
                   tkp = abs(tx - tabulist(i).list);
                   flag = flag & sum(tkp)>0;
               end       
           end
           if flag
              candidate_now.list = tx;
              candidate_now.value = candidate_next(n);
              % 将这个对象放入禁忌表
              for i = tabulength-1:-1:1
                  tabulist(i+1).list = tabulist(i).list;
                  tabulist(i+1).value = tabulist(i).value;
              end
              tabulist(1).list = candidate_now.list;
              tabulist(1).value = candidate_now.value;
              break;
          end
       end
    end
   
