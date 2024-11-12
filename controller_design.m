%clc
%clear all
%close all
n=7;
y=Book1S2(:,1);       %the recorded data from previous runs of the robot
u=Book1S2(:,2);

X0T=[];
for i=n+1:1000
    X0T=[X0T [y(i-n:i-1);u(i-n:i-1)]];
end

PP=randn(size(X0T,1),size(X0T,1));
QQ=randn(size(X0T,2),size(X0T,1));

cvx_begin sdp
variable PP(size(X0T,1),size(X0T,1)) semidefinite
variable QQ(size(X0T,2),size(X0T,1))
[PP X1T*QQ;(X1T*QQ)' PP] >= 0 ;
X0T*QQ == PP;
cvx_end

Kcvx=U0T*(QQ)*(PP)^-1;