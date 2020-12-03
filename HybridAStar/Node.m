classdef Node
    % 3ά��2�׵�״̬�ռ�ڵ�
    % ��¼���ռ�����float���ڵ���[ind]�����ڵ�״̬���[int]
    properties
        parent_node=[];          % ���ڵ��״̬�ռ�-Node
        parent_ctrl=[];
        pos=[0 0 0];        % λ��������
        vel=[0 0 0];        % �ٶ�������
        posInd=[0 0 0];     %
        velInd=[0 0 0];     %
        f=0;                % ���ۺ���
        g=0;                % �Ѿ����ѵĴ���
        h=0;                % ��������ֵ
        t=0;                % ʱ��
    end

    methods
        function obj = Node(pos,vel,parent_node,h,parent_ctrl,Cfg) % ����(λ�á��ٶ�)
            % NODE��Ĺ��캯��
            %   �˴���ʾ��ϸ˵��
            obj.parent_node=parent_node; % ���ڵ�״̬
            obj.pos=pos; % λ��
            % ����ڵ�������դ��������
            obj.posInd=float2ind(pos,Cfg.PosRes,Cfg.posMax);
            obj.velInd=float2ind(vel,Cfg.VelRes,Cfg.velMin);
            %
            if ~isempty(parent_node) % ��һ���ڵ�û��parent_node,��[]����
                obj.h=h;
                obj.g=parent_node.g+Cfg.dT*(1+sum(parent_ctrl.*parent_ctrl));
                obj.f=obj.g+h;
                obj.t=parent_node.t+Cfg.dT;
            end
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 �˴���ʾ�йش˷�����ժҪ
            %   �˴���ʾ��ϸ˵��
            outputArg = obj.Property1 + inputArg;
        end
    end
end
