function K = small_LR(leg_L)
%syms g M R l mw mp Iw L Lm Im Ip;
g =	    9.8011 ; %重力加速度(SZ)
M =  	8.5 ; %机体质量
mw = 	1.436 ;  %驱动轮转子质量
mp = 	1.180 ;   %摆杆质量
Iw =	0.00718 ; %驱动轮转子转动惯量
R =	    0.1 ;  %驱动轮半径
l = 	0.02669 ;   %机体重心到其转轴距离
Im = 0.326125666;%机体绕质心转动惯量
L = -4.8095*leg_L^3+4.4586*leg_L^2-0.8996*leg_L+0.1645;
Lm = 4.9464*leg_L^3-4.5552*leg_L^2+1.9204*leg_L-0.1662;
Ip = -0.2293*leg_L^4+0.2457*leg_L^3+0.3010*leg_L^2+0.0139*leg_L-0.0007;
A1 = (2*Im*Iw*L*g*mp + 2*Im*L*M^2*R^2*g + 2*Im*Lm*M^2*R^2*g + 2*Iw*L*M^2*g*l^2 + 2*Iw*Lm*M^2*g*l^2 + 2*Im*L*R^2*g*mp^2 + 2*Im*Iw*L*M*g + 2*Im*Iw*Lm*M*g + 2*L*M*R^2*g*l^2*mp^2 + 2*L*M^2*R^2*g*l^2*mp + 2*L*M^2*R^2*g*l^2*mw + 2*Lm*M^2*R^2*g*l^2*mp + 2*Lm*M^2*R^2*g*l^2*mw + 4*Im*L*M*R^2*g*mp + 2*Im*L*M*R^2*g*mw + 2*Im*Lm*M*R^2*g*mp + 2*Im*Lm*M*R^2*g*mw + 2*Iw*L*M*g*l^2*mp + 2*Im*L*R^2*g*mp*mw + 2*L*M*R^2*g*l^2*mp*mw)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
A2 = (2*Iw*L*M^2*g*l^2 + 2*Iw*Lm*M^2*g*l^2 + 2*L*M^2*R^2*g*l^2*mw + 2*Lm*M^2*R^2*g*l^2*mp + 2*Lm*M^2*R^2*g*l^2*mw)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
A3 = (2*g*mw*L^2*M^2*R^2*l^2 + 2*Iw*g*L^2*M^2*l^2 + 2*g*mw*L^2*M*R^2*l^2*mp + 2*g*mw*Im*L^2*M*R^2 + 2*Iw*g*L^2*M*l^2*mp + 2*Iw*g*Im*L^2*M + 2*g*mw*Im*L^2*R^2*mp + 2*Iw*g*Im*L^2*mp + 2*g*L*Lm*M^2*R^2*l^2*mp + 4*g*mw*L*Lm*M^2*R^2*l^2 + 4*Iw*g*L*Lm*M^2*l^2 + 2*g*L*Lm*M*R^2*l^2*mp^2 + 2*g*mw*L*Lm*M*R^2*l^2*mp + 2*g*Im*L*Lm*M*R^2*mp + 4*g*mw*Im*L*Lm*M*R^2 + 2*Iw*g*L*Lm*M*l^2*mp + 4*Iw*g*Im*L*Lm*M + 2*g*Im*L*Lm*R^2*mp^2 + 2*g*mw*Im*L*Lm*R^2*mp + 2*Iw*g*Im*L*Lm*mp + 2*g*Lm^2*M^2*R^2*l^2*mp + 2*g*mw*Lm^2*M^2*R^2*l^2 + 2*Iw*g*Lm^2*M^2*l^2 + 2*g*Im*Lm^2*M*R^2*mp + 2*g*mw*Im*Lm^2*M*R^2 + 2*Iw*g*Im*Lm^2*M)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
A4 = (2*Iw*L^2*M^2*g*l^2 + 2*Iw*Lm^2*M^2*g*l^2 + 2*Ip*M^2*R^2*g*l^2 + 2*L^2*M^2*R^2*g*l^2*mw + 2*Lm^2*M^2*R^2*g*l^2*mp + 2*Lm^2*M^2*R^2*g*l^2*mw + 4*Iw*L*Lm*M^2*g*l^2 + 4*L*Lm*M^2*R^2*g*l^2*mw)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
A5 = (2*g*l*mw*L^2*M^2*R^2 + 2*Iw*g*l*L^2*M^2 + 2*g*l*mw*L^2*M*R^2*mp + 2*Iw*g*l*L^2*M*mp + 2*g*l*L*Lm*M^2*R^2*mp + 4*g*l*mw*L*Lm*M^2*R^2 + 4*Iw*g*l*L*Lm*M^2 + 2*g*l*L*Lm*M*R^2*mp^2 + 2*g*l*mw*L*Lm*M*R^2*mp + 2*Iw*g*l*L*Lm*M*mp + 2*g*l*Lm^2*M^2*R^2*mp + 2*g*l*mw*Lm^2*M^2*R^2 + 2*Iw*g*l*Lm^2*M^2)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
A6 = (2*Ip*Iw*M*g*l + 2*Iw*L^2*M^2*g*l + 2*Iw*Lm^2*M^2*g*l + 2*Ip*M^2*R^2*g*l + 2*L^2*M^2*R^2*g*l*mw + 2*Lm^2*M^2*R^2*g*l*mp + 2*Lm^2*M^2*R^2*g*l*mw + 4*Iw*L*Lm*M^2*g*l + 2*Iw*L^2*M*g*l*mp + 2*Ip*M*R^2*g*l*mp + 2*Ip*M*R^2*g*l*mw + 4*L*Lm*M^2*R^2*g*l*mw + 2*L^2*M*R^2*g*l*mp*mw)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);

B1 = -(2*Im*Iw + 2*Im*M*R^2 + 2*Iw*M*l^2 + 2*Im*R^2*mp + 2*Im*R^2*mw + 2*Im*L*M*R + 2*Im*Lm*M*R + 2*M*R^2*l^2*mp + 2*M*R^2*l^2*mw + 2*Im*L*R*mp + 2*L*M*R*l^2*mp)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
B2 = (2*Im*Iw + 2*Im*M*R^2 + 2*Iw*M*l^2 + 2*Im*R^2*mp + 2*Im*R^2*mw + 2*M*R^2*l^2*mp + 2*M*R^2*l^2*mw + 2*Iw*L*M*l + 2*Iw*Lm*M*l + 2*L*M*R^2*l*mw + 2*Lm*M*R^2*l*mp + 2*Lm*M*R^2*l*mw)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
B3 = -(2*Im*Iw*L + 2*Im*Iw*Lm - 2*Im*Ip*R + 2*Iw*L*M*l^2 + 2*Iw*Lm*M*l^2 - 2*Ip*M*R*l^2 + 2*Im*L*R^2*mw + 2*Im*Lm*R^2*mp + 2*Im*Lm*R^2*mw + 2*Im*L*Lm*R*mp + 2*L*M*R^2*l^2*mw + 2*Lm*M*R^2*l^2*mp + 2*Lm*M*R^2*l^2*mw + 2*L*Lm*M*R*l^2*mp)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
B4 = (2*Im*Iw*L + 2*Im*Iw*Lm + 2*Iw*L*M*l^2 + 2*Iw*L^2*M*l + 2*Iw*Lm*M*l^2 + 2*Iw*Lm^2*M*l + 2*Ip*M*R^2*l + 2*Im*L*R^2*mw + 2*Im*Lm*R^2*mp + 2*Im*Lm*R^2*mw + 2*L*M*R^2*l^2*mw + 2*L^2*M*R^2*l*mw + 2*Lm*M*R^2*l^2*mp + 2*Lm^2*M*R^2*l*mp + 2*Lm*M*R^2*l^2*mw + 2*Lm^2*M*R^2*l*mw + 4*Iw*L*Lm*M*l + 4*L*Lm*M*R^2*l*mw)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
B5 = -(2*Iw*L*M*l + 2*Iw*Lm*M*l - 2*Ip*M*R*l + 2*L*M*R^2*l*mw + 2*Lm*M*R^2*l*mp + 2*Lm*M*R^2*l*mw + 2*L*Lm*M*R*l*mp)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
B6 =(2*Ip*Iw + 2*Iw*L^2*M + 2*Iw*Lm^2*M + 2*Ip*M*R^2 + 2*Iw*L^2*mp + 2*Ip*R^2*mp + 2*Ip*R^2*mw + 2*L^2*M*R^2*mw + 2*Lm^2*M*R^2*mp + 2*Lm^2*M*R^2*mw + 4*Iw*L*Lm*M + 2*L^2*R^2*mp*mw + 2*Iw*L*M*l + 2*Iw*Lm*M*l + 4*L*Lm*M*R^2*mw + 2*L*M*R^2*l*mw + 2*Lm*M*R^2*l*mp + 2*Lm*M*R^2*l*mw)/(2*Im*Ip*Iw + 2*Ip*Iw*M*l^2 + 2*Im*Iw*L^2*mp + 2*Im*Ip*R^2*mp + 2*Im*Ip*R^2*mw + 2*Im*Iw*L^2*M + 2*Im*Iw*Lm^2*M + 2*Im*Ip*M*R^2 + 2*Im*L^2*M*R^2*mw + 2*Im*Lm^2*M*R^2*mp + 2*Im*Lm^2*M*R^2*mw + 4*Im*Iw*L*Lm*M + 2*Iw*L^2*M*l^2*mp + 2*Ip*M*R^2*l^2*mp + 2*Ip*M*R^2*l^2*mw + 2*Im*L^2*R^2*mp*mw + 4*Im*L*Lm*M*R^2*mw + 2*L^2*M*R^2*l^2*mp*mw);
A = [0 1 0 0 0 0; A1 0 0 0 A2 0; 0 0 0 1 0 0; A3 0 0 0 A4 0; 0 0 0 0 0 1; A5 0 0 0 A6 0];
B = [0 0; B1 B2; 0 0; B3 B4; 0 0; B5 B6];
[v,d]=eig(A);
D=[B,A*B,A*A*B,A*A*A*B,A*A*A*A*B,A*A*A*A*A*B];
r1=rank(D);

C = eye(6,6);
D = eye(6,2);
E=[C;C*A;C*A*A;C*A*A*A;C*A*A*A*A;C*A*A*A*A*A];
r2=rank(E);
MatQ = [500 0 0 0 0 0; 0 50 0 0 0 0; 0 0 100 0 0 0; 0 0 0 150 0 0; 0 0 0 0 4000 0; 0 0 0 0 0 10]; 
MatR = [1 0; 0 0.25]; %权重矩阵 R 的设计
K=lqr(A,B,MatQ,MatR); %调用 lqr 函数用以求解状态反馈矩阵 K
%取反
K = -K;
end