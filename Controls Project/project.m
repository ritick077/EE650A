clc;clear,close all
%% State Space Model
k=600;
c=10;
m=60;
A = [0 1;-k/m -c/m];
B = [0; 1/m];
C = [1 0];
D = 0;
eigenVal = eig(A);
t=0:0.0001:5;
u=zeros(size(t));
sys = ss(A,B,C,D);
x0=[1 0];
% 
figure(1)
lsim(sys,u,t,x0)
title('State space model of a Cantiliver System')
grid on
%% Controller
Co=ctrb(A,B)
rank(Co)
k=place(A,B,[-10+10i, -10-10i]); %poles for 2 dimensional matrix
sys_cl=ss(A-B*k,B,C,0); 
eig(sys_cl);
% 
figure(2) 
lsim(sys_cl,u,t,x0)
title('Controller 1')
grid on
% 
k1=place(A,B,[-23+20i, -23-20i]); %poles for 2 dimensional matrix
sys_cl2=ss(A-B*k1,B,C,0);
eig(sys_cl2);
% 
figure(3)
lsim(sys_cl2,u,t,x0)
title('Controller 2')
grid on
%% Luenberger Observer
ob=obsv(A,C)
Ro=rank(ob)
l=place(A',C', [-10,-11])'; %k'
% 
at=[A-B*k B*k;zeros(size(A)) A-l*C];
bt=[B; zeros(size(B))];
ct=[C zeros(size(C))];
sys_o=ss(at,bt,ct,0);
%  x1=[0 0]; 
% 
figure(4)
lsim(sys_o,u,t,[x0 x0])
title('Observer')
grid on
%% Error analysis
n=2;
[y,t,x]=lsim(sys_o,u,t,[x0 x0]); 
e=x(:,n+1:end);
x=x(:,1:n);
xe =x-e ; %obs
x1=x(:,1);x2=x(:,2); %plant states
xe1=xe(:,1);xe2=xe(:,2); %observer states
%
figure(5)
plot(t,x1,'-r',t,xe1,'--g',t,x2,'-b',t,xe2,'--y')
title('Error Analysis')
grid on
%