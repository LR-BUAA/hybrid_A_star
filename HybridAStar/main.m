clc;clear;
% config_setup;

Cfg.dT=1; % ʱ�䲽��
Cfg.PosRes=0.2;% λ�÷ֱ���
Cfg.VelRes=0.2;% �ٶȷֱ���
Cfg.AccMax = 2;
Cfg.AccMin =-2;

Cfg.posMin=[0 0 0]';Cfg.posMax=[2 2 2]';
Cfg.velMin=[-2 -2 -2]';Cfg.velMax=[2 2 2]';

% TODO: ��ͼ����
% obstacles=[1,0,0;1,1,0;1,2,0;1,0,1;1,1,1;1,2,1;0,0,1;0,1,1;0,2,1;]; % [n][3]
obstacles=[];   
Cfg.map=zeros((Cfg.posMax'-Cfg.posMin')/Cfg.PosRes);% ���в��ɴ�λ��
for obs_=obstacles'
%   Cfg.map([ceil(obs_(1)/Cfg.PosRes):ceil((obs_(1)+1)/Cfg.PosRes)],[ceil(obs_(2)/Cfg.PosRes):ceil((obs_(2)+1)/Cfg.PosRes)],[ceil(obs_(3)/Cfg.PosRes):ceil((obs_(3)+1)/Cfg.PosRes)])=1;
%   Cfg.map([float2ind(obs_(1),Cfg.PosRes,Cfg.posMin),float2ind(obs_(1)+1,Cfg.PosRes,Cfg.posMin)],[float2ind(obs_(2),Cfg.PosRes,Cfg.posMin),float2ind(obs_(2)+1,Cfg.PosRes,Cfg.posMin)],[float2ind(obs_(3),Cfg.PosRes,Cfg.posMin),float2ind(obs_(3)+1,Cfg.PosRes,Cfg.posMin)])=1;
   posInd_L=float2ind(obs_,Cfg.PosRes,Cfg.posMin);
   posInd_U=float2ind(obs_+1,Cfg.PosRes,Cfg.posMin);
   Cfg.map(posInd_L(1):posInd_U(1),posInd_L(2):posInd_U(2),posInd_L(3):posInd_U(3))=1;
end

start_state  =[0,0,0,0,0,0]';
goal_state   =[2,2,2,0,0,0]';

[xs,us]=hybridAStar(start_state,goal_state,Cfg);
