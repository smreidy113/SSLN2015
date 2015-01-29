x = 40;
y = 60;
zp = 50;

z = sqrt(y^2+zp^2);

l1 = 195.11;
l2 = 183.73;
l3 = 116.49;

d3 = (l1^2-l2^2)/(2*x);

d1 = (l2^2-l3^2+z^2-1/4*x^2+d3*x)/(2*z);

d2 = sqrt(l3^2-d3^2-(d1-z)^2);

d = [d1 d2 d3]'

distb = sqrt(d1^2+d2^2+d3^2);

t = atan2(zp,y);

R01 = [cos(t) sin(t) 0;
       -sin(t) cos(t) 0;
       0 0 1];
   
dp = R01*d;

dist = sqrt(dp(1)^2+dp(3)^2)
theta = atan2d(dp(3),dp(1))