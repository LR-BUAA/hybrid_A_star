classdef NodeList<handle
    % ���Node��
    % ����Open��Close
    
    properties
        list=[];
        Cfg=struct();
    end
    
    methods
        function obj = NodeList(Cfg)
            %OPENLIST ��������ʵ��
            %   �˴���ʾ��ϸ˵��
            obj.Cfg=Cfg;
        end
        
        function add(obj,new_node,unique_flg)
            flg_given=exist('unique_flg');
            if ~flg_given
                unique_flg=true;
            end
            %add ��list����ӷ��ظ��ĵ�
            %  ����������Node
            %  �ظ����鼰�߽����
            bool=false;
            [is_in_list ind,node_]=obj.isInList(new_node);
            if any(new_node.pos<obj.Cfg.posMin) || any(new_node.pos>obj.Cfg.posMax) || any(new_node.vel<obj.Cfg.velMin) || any(new_node.vel>obj.Cfg.velMax)% �߽���
                return
            end
            if is_in_list && unique_flg
            % ������ظ�״̬����Ƚϴ��ۺ���
                if new_node.f<node_.f % �µıȾɵĸ���
                    obj.list(ind)=new_node;%ֱ��ȡ��
                end
            else
                obj.list=[obj.list,new_node];
            end
        end

        function [bool node]=pop(obj,ins)
            % �Ӷ����е���һ��ָ����node
            bool=false;node=0;
            if isempty(obj.list)
                % ���ListΪ����ֱ�ӷ��ؿ�
                return
            end
            if ins=='min'
                % �������ۺ�����С��Node
                [~,ind]=min([obj.list.f]);
                node=obj.list(ind);bool=true;
                obj.list(ind)=[];
            elseif isa(ins,'double')
                %�����������λ�������򵯳���Ӧλ��
                if 0<ins && ins<length(obj.list)
                    % �������ȷ�Χ
                    return
                end
                node=obj.list(ins);bool=true;
            end
        end
        
        function [bool,ind,node_return]=isInList(obj,node)
            % �ж�Node�Ƿ���List��
            bool=false;cnt=1;ind=0;node_return=0;
            for node_=obj.list
                % �Ա�posInd,velInd
                if all(node_.posInd==node.posInd) && all(node_.velInd==node.velInd)
                    bool=true;
                    ind=cnt;
                    node_return=node_;
                    return
                end
                cnt=cnt+1;
            end
            % û�ҵ������·���,bool��node����0
        end
    end
end
