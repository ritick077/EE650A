clc;clear,close all
a=[0 1 0;980 0 -2.8;0 0 -100]; 
b=[0; 0 ; 100];
c=[1 0 0];
eig(a)
t=0:0.01:1;
u=zeros(size(t));
sys=ss(a,b,c,0);
x0=[1 0 0];

figure(1)
lsim(sys,u,t,x0)

Co=ctrb(a,b);
rank(Co)
k=place(a,b,[-10+10i, -10-10i, -50]); %poles for 3 dimensional matrix
sys_cl=ss(a-b*k,b,c,0); 
eig(sys_cl)

figure(2) 
lsim(sys_cl,u,t,x0)

k1=place(a,b, [-20+20i,-20-20i,-70]);
sys_cl2=ss(a-b*k1,b,c,0);

figure(3)

eig(sys_cl2);
lsim(sys_cl2,u,t,x0)

ob=obsv(a,c);
Ro=rank(ob);
l=place(a',c', [-100,-101,-102])' %k'

at=[a-b*k b*k;zeros(size(a)) a-l*c];
bt=[b; zeros(size(b))];
ct=[c zeros(size(c))];
sys_o=ss(at,bt,ct,0);
x1=[0.01 0.5 -5]; 

figure(4)
lsim(sys_o,u,t,[x0 x0])

n=3;
[y,t,x]=lsim(sys_o,u,t,[x0 x0]); 
e=x(:,n+1:end);
x=x(:,1:n);
xe =x-e ; %obs
x1=x(:,1);x2=x(:,2);x3=x(:,3); %plant states
xe1=xe(:,1);xe2=xe(:,2);xe3=xe(:,3); %observer states

figure(5)

plot(t,x1,'-r',t,xe1,'--r',t,x2,'-b',t,xe2,'--b',t,x3,'k',t,xe3,'-k')