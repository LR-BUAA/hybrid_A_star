classdef NodeList<handle
    % 存放Node类
    % 用于Open和Close
    
    properties
        list=[];
        Cfg=struct();
    end
    
    methods
        function obj = NodeList(Cfg)
            %OPENLIST 构造此类的实例
            %   此处显示详细说明
            obj.Cfg=Cfg;
        end
        
        function add(obj,new_node,unique_flg)
            flg_given=exist('unique_flg');
            if ~flg_given
                unique_flg=true;
            end
            %add 向list中添加非重复的点
            %  输入待插入的Node
            %  重复检验及边界检验
            bool=false;
            [is_in_list ind,node_]=obj.isInList(new_node);
            if any(new_node.pos<obj.Cfg.posMin) || any(new_node.pos>obj.Cfg.posMax) || any(new_node.vel<obj.Cfg.velMin) || any(new_node.vel>obj.Cfg.velMax)% 边界检测
                return
            end
            if is_in_list && unique_flg
            % 如果有重复状态点则比较代价函数
                if new_node.f<node_.f % 新的比旧的更优
                    obj.list(ind)=new_node;%直接取代
                end
            else
                obj.list=[obj.list,new_node];
            end
        end

        function [bool node]=pop(obj,ins)
            % 从队列中弹出一个指定的node
            bool=false;node=0;
            if isempty(obj.list)
                % 如果List为空则直接返回空
                return
            end
            if ins=='min'
                % 弹出代价函数最小的Node
                [~,ind]=min([obj.list.f]);
                node=obj.list(ind);bool=true;
                obj.list(ind)=[];
            elseif isa(ins,'double')
                %　如果给整数位置索引则弹出对应位置
                if 0<ins && ins<length(obj.list)
                    % 超出长度范围
                    return
                end
                node=obj.list(ins);bool=true;
            end
        end
        
        function [bool,ind,node_return]=isInList(obj,node)
            % 判定Node是否在List中
            bool=false;cnt=1;ind=0;node_return=0;
            for node_=obj.list
                % 对比posInd,velInd
                if all(node_.posInd==node.posInd) && all(node_.velInd==node.velInd)
                    bool=true;
                    ind=cnt;
                    node_return=node_;
                    return
                end
                cnt=cnt+1;
            end
            % 没找到则无事发生,bool和node返回0
        end
    end
end
