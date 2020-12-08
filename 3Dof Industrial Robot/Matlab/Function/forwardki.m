function [X,Y,Z]=forwardki(a,b,c,params)
l1 = params.l1;
l2 = params.l2;
X=-(l1*cos(a)+l2.*cos(a+b)).*sin(c);
Y=(l1*cos(a)+l2.*cos(a+b)).*cos(c);
Z=l1*sin(a)+l2.*sin(a+b);

