function k = Balance_Luntuimatlab(leg_length)
    %theta : 摆杆与竖直方向夹角             R   ：驱动轮半径
    %x     : 驱动轮位移                    L   : 摆杆重心到驱动轮轴距离
    %phi   : 机体与水平夹角                Lm  : 摆杆重心到其转轴距离
    %T     ：驱动轮输出力矩                 l   : 机体重心到其转轴距离
    % Tp    : 髋关节输出力矩                 mw  : 驱动轮转子质量
    %N     ：驱动轮对摆杆力的水平分量        mp  : 摆杆质量
    %P     ：驱动轮对摆杆力的竖直分量        M   : 机体质量r
    %Nm    ：摆杆对机体力水平方向分量        Iw  : 驱动轮转子转动惯量
    %Pm    ：摆杆对机体力竖直方向分量        Ip  : 摆杆绕质心转动惯量
    %Nf    : 地面对驱动轮摩擦力             Im  : 机体绕质心转动惯量

    syms x0(t) T R Iw mw M L Lm theta0(t) l phi0(t) mp g Tp Ip Im 
    syms d2_theta d2_phi d2_x d_theta d_x d_phi theta x phi 


    %leg_length = 0.18;
    R1=0.055;                          %驱动轮半径
    L1=leg_length/2;                  %摆杆重心到驱动轮轴距离
    LM1=leg_length/2;                 %摆杆重心到其转轴距离
    l1=0.0011;                          %机体质心距离转轴距离
    mw1=0.88;                         %驱动轮质量
    mp1=1.12;                         %杆质量
    M1=18.12;                          %机体质量
    Iw1=mw1*R1^2;                     %驱动轮转动惯量
    Ip1=(mp1*((L1+LM1)^2));%/3.0);%*10;     %摆杆转动惯量
    IM1=M1*((0.109^2));%/3.0);%*5;     %机体绕质心转动惯量

    Nm = M*diff(x0 + (L + Lm )*sin(theta0)-l*sin(phi0),t,2);
    N = Nm + mp*diff(x0 + L*sin(theta0),t,2);

    Pm = M*g + M*diff((L+Lm)*cos(theta0)+l*cos(phi0),t,2);
    P = Pm +mp*g+mp*diff(L*cos(theta0),t,2);

    eqn1 = diff(x0,t,2) == (T -N*R)/(Iw/R + mw*R);
    eqn2 = Ip*diff(theta0,t,2) == (P*L + Pm*Lm)*sin(theta0)-(N*L+Nm*Lm)*cos(theta0)-T+Tp;
    eqn3 = Im*diff(phi0,t,2) == Tp +Nm*l*cos(phi0)+Pm*l*sin(phi0);

    eqn10 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn1,diff(theta0,t,2),d2_theta),diff(x0,t,2),d2_x),diff(phi0,t,2),d2_phi),diff(theta0,t),d_theta),diff(x0,t),d_x),diff(phi0,t),d_phi),theta0,theta),x0,x),phi0,phi);
    eqn20 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn2,diff(theta0,t,2),d2_theta),diff(x0,t,2),d2_x),diff(phi0,t,2),d2_phi),diff(theta0,t),d_theta),diff(x0,t),d_x),diff(phi0,t),d_phi),theta0,theta),x0,x),phi0,phi);
    eqn30 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn3,diff(theta0,t,2),d2_theta),diff(x0,t,2),d2_x),diff(phi0,t,2),d2_phi),diff(theta0,t),d_theta),diff(x0,t),d_x),diff(phi0,t),d_phi),theta0,theta),x0,x),phi0,phi);


    [d2_theta,d2_x,d2_phi] = solve(eqn10,eqn20,eqn30,d2_theta,d2_x,d2_phi);

    A=subs(jacobian([d_theta,d2_theta,d_x,d2_x,d_phi,d2_phi],[theta,d_theta,x,d_x,phi,d_phi]),[theta,d_theta,d_x,phi,d_phi,T,Tp],[0,0,0,0,0,0,0]);
    A=subs(A,[R,L,Lm,l,mw,mp,M,Iw,Ip,Im,g],[R1,L1,LM1,l1,mw1,mp1,M1,Iw1,Ip1,IM1,9.83]);
    A=double(A);
    B=subs(jacobian([d_theta,d2_theta,d_x,d2_x,d_phi,d2_phi],[T,Tp]),[theta,d_theta,d_x,phi,d_phi,T,Tp],[0,0,0,0,0,0,0]);
    B=subs(B,[R,L,Lm,l,mw,mp,M,Iw,Ip,Im,g],[R1,L1,LM1,l1,mw1,mp1,M1,Iw1,Ip1,IM1,9.83]);
    B=double(B);


    Q=diag([30 1 500 100 5000 1]);%theta d_theta x d_x phi d_phi%700 1 600 200 1000 1 //1 1 500 100 5000 1//600 1 500 150 1000 1
    R=diag([1  0.25]);                %T Tp

    k=lqr(A,B,Q,R);



% function k = Balance_Luntuimatlab(leg_length)
% %theta : 摆杆与竖直方向夹角 R ：驱动轮半径
% %x : 驱动轮位移 L : 摆杆重心到驱动轮轴距离
% %phi : 机体与水平夹角 Lm : 摆杆重心到其转轴距离
% %T ：驱动轮输出力矩 l : 机体重心到其转轴距离
% %Tp : 髋关节输出力矩 mw : 驱动轮转子质量
% %N ：驱动轮对摆杆力的水平分量 mp : 摆杆质量
% %P ：驱动轮对摆杆力的竖直分量 M : 机体质量
% %Nm ：摆杆对机体力水平方向分量 Iw : 驱动轮转子转动惯量
% %Pm ：摆杆对机体力竖直方向分量 Ip : 摆杆绕质心转动惯量
% %Nf : 地面对驱动轮摩擦力 Im : 机体绕质心转动惯量
% syms x0(t) T R Iw mw M L Lm theta0(t) l phi0(t) mp g Tp Ip Im 
% syms d2_theta d2_phi d2_x d_theta d_x d_phi theta x phi 
% R1=0.055; %驱动轮半径
% L1=leg_length/2; %摆杆重心到驱动轮轴距离
% LM1=leg_length/2; %摆杆重心到其转轴距离
% l1=0.011; %机体质心距离转轴距离
% mw1=0.9; %驱动轮质量
% mp1=1.24; %杆质量
% M1=11; %机体质量
% Iw1=mw1*R1^2; %驱动轮转动惯量
% Ip1=(mp1*((leg_length)^2)/12.0); %摆杆转动惯量
% IM1=M1*((0.45^2+0.23^2))/12; %机体绕质心转动惯量
% Nm = M*diff(x0 + (L + Lm )*sin(theta0)-l*sin(phi0),t,2);
% N = Nm + mp*diff(x0 + L*sin(theta0),t,2);
% Pm = M*g + M*diff((L+Lm)*cos(theta0)+l*cos(phi0),t,2);
% P = Pm +mp*g+mp*diff(L*cos(theta0),t,2);
% eqn1 = diff(x0,t,2) == (T -N*R)/(Iw/R + mw*R);
% eqn2 = Ip*diff(theta0,t,2) == (P*L + Pm*Lm)*sin(theta0)-(N*L+Nm*Lm)*cos(theta0)-T+Tp;
% eqn3 = Im*diff(phi0,t,2) == Tp +Nm*l*cos(phi0)+Pm*l*sin(phi0);
% eqn10 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn1,diff(theta0,t,2),d2_theta),diff(x0,t,2),d2_x),diff(phi0,t,2),d2_phi),diff(theta0,t),d_theta),diff(x0,t),d_x),diff(phi0,t),d_phi),theta0,theta),x0,x),phi0,phi);
% eqn20 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn2,diff(theta0,t,2),d2_theta),diff(x0,t,2),d2_x),diff(phi0,t,2),d2_phi),diff(theta0,t),d_theta),diff(x0,t),d_x),diff(phi0,t),d_phi),theta0,theta),x0,x),phi0,phi);
% eqn30 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn3,diff(theta0,t,2),d2_theta),diff(x0,t,2),d2_x),diff(phi0,t,2),d2_phi),diff(theta0,t),d_theta),diff(x0,t),d_x),diff(phi0,t),d_phi),theta0,theta),x0,x),phi0,phi);
% [d2_theta,d2_x,d2_phi] = solve(eqn10,eqn20,eqn30,d2_theta,d2_x,d2_phi);
% A=subs(jacobian([d_theta,d2_theta,d_x,d2_x,d_phi,d2_phi],[theta,d_theta,x,d_x,phi,d_phi]),[theta,d_theta,d_x,phi,d_phi,T,Tp],[0,0,0,0,0,0,0]);
% A=subs(A,[R,L,Lm,l,mw,mp,M,Iw,Ip,Im,g],[R1,L1,LM1,l1,mw1,mp1,M1,Iw1,Ip1,IM1,9.8]);
% A=double(A);
% B=subs(jacobian([d_theta,d2_theta,d_x,d2_x,d_phi,d2_phi],[T,Tp]),[theta,d_theta,d_x,phi,d_phi,T,Tp],[0,0,0,0,0,0,0]);
% B=subs(B,[R,L,Lm,l,mw,mp,M,Iw,Ip,Im,g],[R1,L1,LM1,l1,mw1,mp1,M1,Iw1,Ip1,IM1,9.8]);
% B=double(B);
% sys = ss(A,B,[],[]);
% Ts = 0.001;
% sys_d = c2d(sys, Ts); 
% A = sys_d.A;
% B = sys_d.B;
% Q=diag([30 1 500 100 5000 1]);
% R=diag([0.75 0.25]); 
% k=dlqr(A,B,Q,R);
% end 
    