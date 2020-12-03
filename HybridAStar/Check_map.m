X=linspace(0,10,100);
Y=linspace(0,10,100);
Z=linspace(0,10,100);
[X,Y,Z]=meshgrid(X,Y,Z);
% Z(Cfg.map==0)=0;
X=X(Cfg.map==1);
Y=Y(Cfg.map==1);
Z=Z(Cfg.map==1);
scatter3(X,Y,Z,'k');
