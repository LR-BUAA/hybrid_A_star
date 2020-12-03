%%
startx = [0,2,0];
starty = [0,0,0];
startz = [0,0,0];

finalx = [3,3,0];
finaly = [2,0,0];
finalz = [1,0,0];

func = @(T)cost(T,startx,finalx,starty,finaly,startz,finalz);
T = fminbnd(func,0,10)
cost(T,startx,finalx,starty,finaly,startz,finaly)

paramx = calPara(startx,finalx,T);
paramy = calPara(starty,finaly,T);
paramz = calPara(startz,finalz,T);



for t = 0:0.01:T
    px = calValue(paramx,startx,t);
    py = calValue(paramy,starty,t);
    pz = calValue(paramz,startz,t);
    plot3(px,py,pz,'r.');
    hold on;
    grid on
end

function value = calValue(param,start,t)
    value = 1/120*param(1)*t^5 + 1/24*param(2)*t^4+1/6*param(3)*t^3+1/2*start(3)*t^2+start(2)*t+start(1);
end

function J = cost(T,startx,finalx,starty,finaly,startz,finalz)
    param = calPara(startx,finalx,T);
    alpha = param(1);
    belta = param(2);
    gama = param(3);
    % 使用初态S0、终态Sf和持续时间T解出待定系数之后，即可给出控制变量u(t)和状态变量S(t)
    J1 = gama^2 + gama*belta*T +1/3*belta^2*T^2 + 1/3*alpha*gama*T^2 + 1/4*alpha*belta*T^3+1/20*alpha^2*T^4;
    
    param = calPara(starty,finaly,T);
    alpha = param(1);
    belta = param(2);
    gama = param(3);
    J2 = gama^2 + gama*belta*T +1/3*belta^2*T^2 + 1/3*alpha*gama*T^2 + 1/4*alpha*belta*T^3+1/20*alpha^2*T^4;
    
    param = calPara(startz,finalz,T);
    alpha = param(1);
    belta = param(2);
    gama = param(3);
    J3 = gama^2 + gama*belta*T +1/3*belta^2*T^2 + 1/3*alpha*gama*T^2 + 1/4*alpha*belta*T^3+1/20*alpha^2*T^4;
    
    J = J1+J2+J3;
end

function param = calPara(start,final,T)
    % 给定初态S、终态S、持续时间T
    % 解出待定系数
    dp = final(1) - start(1) - start(2)*T - 0.5*start(3)*T^2;
    dv = final(2) - start(2) - start(3)*T;
    da = final(3) - start(3);
    
    A = [  720 ,  -360*T,  60*T^2;
         -360*T, 168*T^2, -24*T^3;
         60*T^2, -24*T^3,   3*T^4];
     
    param = 1/T^5*A*[dp;dv;da];
end