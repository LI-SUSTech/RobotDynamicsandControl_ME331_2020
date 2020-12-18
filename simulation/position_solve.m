function equ=position_solve(para)
L1=180;
L2=150;
L3=50;
H=100;
global X;
global Y;
global Z;
equ(1)=-X+sin(para(3))*(L1*cos(para(1))+L2*cos(para(2))+L3);
equ(3)=-Y+cos(para(3))*(L1*cos(para(1))+L2*cos(para(2))+L3);
equ(3)=-Z+H+L1*sin(para(1))-L2*sin(para(2));
end
