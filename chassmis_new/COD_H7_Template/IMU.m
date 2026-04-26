syms q0 q1 q2 q3;
syms gx gy gz bx by; 
syms dt;
X = [q0;q1;q2;q3;bx;by];
Wx = gx - bx;
Wy = gy - by;
Wz = gz;
Omega = [ 0  -Wx  -Wy  -Wz;
          Wx  0    Wz  -Wy;                    
          Wy -Wz   0    Wx;
          Wz  Wy  -Wx   0 ;];
q = [q0;q1;q2;q3];

f = [q + 0.5*(Omega)*q;
           bx         ;       
           by         ;];

F = simplify(jacobian(f,X));

Ok = F(1:4,5:6);

C = [  1 - 2*(q2^2 + q3^2)  ,    2*(q1*q2 - q0*q3)   ,   2*(q1*q3 + q0*q2) ; 
       2*(q1*q2 + q0*q3)    ,   1 - 2*(q1^2 + q3^2)  ,   2*(q2*q3 - q0*q1) ;            
       2*(q1*q3 - q0*q2)    ,    2*(q2*q3 + q0*q1)   ,  1 - 2*(q1^2 + q3^2); ];
g = transpose(C(3,1:3));

h = transpose([2*(q1*q3 - q0*q2) , 2*(q2*q3 + q0*q1) , (q0^2 - q1^2 -q2^2 +q3^2)]);

H = simplify(jacobian(h,X));

M = diag(ones(1,6),0);
M(4,4) = 0;
disp(M);









