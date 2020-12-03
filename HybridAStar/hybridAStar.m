function [return_flag,state_history,ctl_history]=hybridAStar(start_state,goal_state,Cfg)
    %
    % return_flag: 0-寻路失败；1-寻路成功；
    state_history=[start_state];% n*6

    A=[0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1;0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0];
    B=[zeros(3);eye(3)];
    C=[eye(3),zeros(3)];% 观测位置
    D=zeros(3);
    ss_= ss(A,B,C,D);% 动力学模型

    % 控制量离散化
    ax=linspace(Cfg.AccMin,Cfg.AccMax,5);
    ay=linspace(Cfg.AccMin,Cfg.AccMax,5);
    az=linspace(Cfg.AccMin,Cfg.AccMax,5);
    [ax,ay,az]=meshgrid(ax,ay,az);
    ax=ax(:);ay=ay(:);az=az(:);

    % 创建OpenList与CloseList
    Open=NodeList(Cfg);
    Close=NodeList(Cfg);
    % 初始化
    start_node=Node(start_state(1:3),start_state(4:6),[],getHeuristic(start_state,goal_state),0,Cfg);
    Open.add(start_node);

    while ~isempty(Open.list)
        % OpenList中弹出代价函数最小的点
        [node_exist,wk_node]=Open.pop('min'); % 弹出节点
        Close.add(wk_node,false); % 加入CloseList
        % TODO:到达目标的判定
        if norm(wk_node.pos-goal_state(1:3))<0.1 && norm(wk_node.vel-goal_state(4:6))
            return_flag=1;
            break
        end
        % TODO：CloseList添加
        % 生成子节点
        for i=1:length(ax) % 遍历控制量生成子节点
            ctr=[ax(i) ay(i) az(i)]';
            % 求取子节点状态变量
            [y1,t1,state_now]=lsim(ss_,[ctr ctr],[0,Cfg.dT],[wk_node.pos,wk_node.vel]');
            y1=y1(end,:);t1=wk_node.t+Cfg.dT;state_now=state_now(end,:);

            %　计算代价函数
            h=getHeuristic(start_state,start_state);% h-启发值：无视障碍物的最优控制
            % g-已用路径的代价：\int^T_0 1+u^Tu dt
%            g=wk_node.g+(1+ctr'*ctr)*t1;
%            f=g+h;
            % 尝试插入OpenList
            child_node=Node(state_now(1:3),state_now(4:6),wk_node,h,ctr,Cfg);
            Open.add(child_node);
        end
    end

    if return_flag
        % TODO:反向寻路

    end
end

function h = getHeuristic(state_now,state_goal)
    % h-启发值：无视障碍物的最优控制的代价函数
    % start_pos=state_now(1:3);   start_vel=state_now(4:6);
    % end_pos = state_goal(1:3);    end_vel=state_goal(4:6);
    % 找寻T使得J_T最优
    J_T=@(T)cost(state_now,state_goal,T); % 指定全程时间T为唯一自变量
    T_opt=fminbnd(J_T,0,10);
    Jmin=cost(state_now,state_goal,T_opt);
    h=Jmin;
end

 function J=cost(start_state,goal_state,T)
     constans=zeros(6,1);
     ds=goal_state(:)-start_state(:)-[start_state(4:6)*T 0 0 0]';
     A=[-12/(T*T*T)*eye(3),6/(T*T)*eye(3);6/(T*T)*eye(3),-2/T*eye(3)];
     constans=A*ds;
     a1=constans(1);a2=constans(2);a3=constans(3);
     b1=constans(4);b2=constans(5);b3=constans(6);
     J=T+(1/3*a1*a1*T*T*T+a1*b1*T*T+b1*b1*T)+(1/3*a2*a2*T*T*T+a2*b2*T*T+b2*b2*T)+(1/3*a3*a3*T*T*T+a3*b3*T*T+b3*b3*T);
 end