classdef Node
    % 3维、2阶的状态空间节点
    % 记录：空间坐标float、节点编号[ind]、父节点状态编号[int]
    properties
        parent_node=[];          % 父节点的状态空间-Node
        parent_ctrl=[];
        pos=[0 0 0];        % 位置行向量
        vel=[0 0 0];        % 速度行向量
        posInd=[0 0 0];     %
        velInd=[0 0 0];     %
        f=0;                % 代价函数
        g=0;                % 已经花费的代价
        h=0;                % 启发函数值
        t=0;                % 时间
    end

    methods
        function obj = Node(pos,vel,parent_node,h,parent_ctrl,Cfg) % 二阶(位置、速度)
            % NODE类的构造函数
            %   此处显示详细说明
            obj.parent_node=parent_node; % 父节点状态
            obj.pos=pos; % 位置
            % 计算节点所处的栅格（整数）
            obj.posInd=float2ind(pos,Cfg.PosRes,Cfg.posMax);
            obj.velInd=float2ind(vel,Cfg.VelRes,Cfg.velMin);
            %
            if ~isempty(parent_node) % 第一个节点没有parent_node,用[]代替
                obj.h=h;
                obj.g=parent_node.g+Cfg.dT*(1+sum(parent_ctrl.*parent_ctrl));
                obj.f=obj.g+h;
                obj.t=parent_node.t+Cfg.dT;
            end
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.Property1 + inputArg;
        end
    end
end
