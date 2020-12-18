function angle=solve_angle(position) 
global X;
global Y;
global Z;
X=position(1);
Y=position(2);
Z=position(3);
opts=optimset('Display','iter','TolFun',1e-8);

init=[pi/4,pi/4,pi/4];
result=fsolve(@position_solve,init,opts);
angle=result;
end
