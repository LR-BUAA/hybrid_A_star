function [return_flag,state_history,ctl_history]=hybridAStar(start_state,goal_state,Cfg)
    %
    % return_flag: 0-Ѱ·ʧ�ܣ�1-Ѱ·�ɹ���
    state_history=[start_state];% n*6

    A=[0 0 0 1 0 0;0 0 0 0 1 0;0 0 0 0 0 1;0 0 0 0 0 0;0 0 0 0 0 0;0 0 0 0 0 0];
    B=[zeros(3);eye(3)];
    C=[eye(3),zeros(3)];% �۲�λ��
    D=zeros(3);
    ss_= ss(A,B,C,D);% ����ѧģ��

    % ��������ɢ��
    ax=linspace(Cfg.AccMin,Cfg.AccMax,5);
    ay=linspace(Cfg.AccMin,Cfg.AccMax,5);
    az=linspace(Cfg.AccMin,Cfg.AccMax,5);
    [ax,ay,az]=meshgrid(ax,ay,az);
    ax=ax(:);ay=ay(:);az=az(:);

    % ����OpenList��CloseList
    Open=NodeList(Cfg);
    Close=NodeList(Cfg);
    % ��ʼ��
    start_node=Node(start_state(1:3),start_state(4:6),[],getHeuristic(start_state,goal_state),0,Cfg);
    Open.add(start_node);

    while ~isempty(Open.list)
        % OpenList�е������ۺ�����С�ĵ�
        [node_exist,wk_node]=Open.pop('min'); % �����ڵ�
        Close.add(wk_node,false); % ����CloseList
        % TODO:����Ŀ����ж�
        if norm(wk_node.pos-goal_state(1:3))<0.1 && norm(wk_node.vel-goal_state(4:6))
            return_flag=1;
            break
        end
        % TODO��CloseList���
        % �����ӽڵ�
        for i=1:length(ax) % ���������������ӽڵ�
            ctr=[ax(i) ay(i) az(i)]';
            % ��ȡ�ӽڵ�״̬����
            [y1,t1,state_now]=lsim(ss_,[ctr ctr],[0,Cfg.dT],[wk_node.pos,wk_node.vel]');
            y1=y1(end,:);t1=wk_node.t+Cfg.dT;state_now=state_now(end,:);

            %��������ۺ���
            h=getHeuristic(start_state,start_state);% h-����ֵ�������ϰ�������ſ���
            % g-����·���Ĵ��ۣ�\int^T_0 1+u^Tu dt
%            g=wk_node.g+(1+ctr'*ctr)*t1;
%            f=g+h;
            % ���Բ���OpenList
            child_node=Node(state_now(1:3),state_now(4:6),wk_node,h,ctr,Cfg);
            Open.add(child_node);
        end
    end

    if return_flag
        % TODO:����Ѱ·

    end
end

function h = getHeuristic(state_now,state_goal)
    % h-����ֵ�������ϰ�������ſ��ƵĴ��ۺ���
    % start_pos=state_now(1:3);   start_vel=state_now(4:6);
    % end_pos = state_goal(1:3);    end_vel=state_goal(4:6);
    % ��ѰTʹ��J_T����
    J_T=@(T)cost(state_now,state_goal,T); % ָ��ȫ��ʱ��TΪΨһ�Ա���
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