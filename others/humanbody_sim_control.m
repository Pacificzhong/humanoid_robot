%% forward kinematic
clear all
m_l3=2;
m_u1=2;
m_u2=0.5;
m_u4=0.2;

l_l1=0.5;
l_l2=0.5;
l_l3=0.2;
l_u1=0.5;
l_u2=0.5;
l_u3=0.1;
l_u4=0.1;

q_l1=120/180*pi;
q_l2=60/180*pi;
q_l3=90/180*pi;
q_u1=110/180*pi;
q_u2=120/180*pi;
q_u3=210/180*pi;
q_u4=230/180*pi;

x_l1=0;
y_l1=0;
x_l2=x_l1+l_l1*cos(q_l1);
y_l2=y_l1+l_l1*sin(q_l1);
x_l3=x_l2+l_l2*cos(q_l2);
y_l3=y_l2+l_l2*sin(q_l2);
x_u1=x_l3+l_l3*cos(q_l3);
y_u1=y_l3+l_l3*sin(q_l3);
x_u2=x_u1+l_u1*cos(q_u1);
y_u2=y_u1+l_u1*sin(q_u1);
x_u3=x_u2+l_u2*cos(q_u2);
y_u3=y_u2+l_u2*sin(q_u2);
x_u4=x_u3+l_u3*cos(q_u3);
y_u4=y_u3+l_u3*sin(q_u3);
x_u5=x_u4+l_u4*cos(q_u4);
y_u5=y_u4+l_u4*sin(q_u4);

dt=1/20;
for i=1:1*1/dt

J=[-l_l1*sin(q_l1) -l_l2*sin(q_l2) 0 0 0 0 0;...
   -l_l1*sin(q_l1) -l_l2*sin(q_l2) -l_l3*sin(q_l3) -l_u1*sin(q_u1) -l_u2*sin(q_u2) -l_u3*sin(q_u3) 0;...   
    l_l1*cos(q_l1)  l_l2*cos(q_l2)  l_l3*cos(q_l3)  l_u1*cos(q_u1)  l_u2*cos(q_u2)  l_u3*cos(q_u3) 0;...
    -l_l1*sin(q_l1) -l_l2*sin(q_l2) -l_l3*sin(q_l3) -l_u1*sin(q_u1) -l_u2*sin(q_u2) -l_u3*sin(q_u3)  -l_u4*sin(q_u4);...   
    l_l1*cos(q_l1)  l_l2*cos(q_l2)  l_l3*cos(q_l3)  l_u1*cos(q_u1)  l_u2*cos(q_u2)  l_u3*cos(q_u3)  l_u4*cos(q_u4)];

x_r_u5=-0.5;  
y_r_u5=1.5;
theta_r_u5=0;
x_r_u4=x_r_u5+l_u4*cos(theta_r_u5);  
y_r_u4=y_r_u5+l_u4*sin(theta_r_u5);
x_r_l3=-1/m_l3*(m_u1*x_u1+m_u2*x_u2+m_u4*x_u4);

e=[x_l3-x_r_l3,x_u4-x_r_u4,y_u4-y_r_u4,x_u5-x_r_u5,y_u5-y_r_u5]';
W_weigt=diag([1 1 1 10 10 10 10]);
d_q=-W_weigt*pinv(J)*e;
d_q_l1=d_q(1);d_q_l2=d_q(2);
d_q_l3=d_q(3);d_q_u1=d_q(4);
d_q_u2=d_q(5);d_q_u3=d_q(6);
d_q_u4=d_q(7);

q_l1=q_l1+dt*d_q_l1;
q_l2=q_l2+dt*d_q_l2;
q_l3=q_l3+dt*d_q_l3;
q_u1=q_u1+dt*d_q_u1;
q_u2=q_u2+dt*d_q_u2;
q_u3=q_u3+dt*d_q_u3;
q_u4=q_u4+dt*d_q_u4;

x_l2=x_l1+l_l1*cos(q_l1);
y_l2=y_l1+l_l1*sin(q_l1);
x_l3=x_l2+l_l2*cos(q_l2);
y_l3=y_l2+l_l2*sin(q_l2);
x_u1=x_l3+l_l3*cos(q_l3);
y_u1=y_l3+l_l3*sin(q_l3);
x_u2=x_u1+l_u1*cos(q_u1);
y_u2=y_u1+l_u1*sin(q_u1);
x_u3=x_u2+l_u2*cos(q_u2);
y_u3=y_u2+l_u2*sin(q_u2);
x_u4=x_u3+l_u3*cos(q_u3);
y_u4=y_u3+l_u3*sin(q_u3);
x_u5=x_u4+l_u4*cos(q_u4);
y_u5=y_u4+l_u4*sin(q_u4);

x_m=1/(m_l3+m_u1+m_u2+m_u4)*(m_l3*x_l3+m_u1*x_u1+m_u2*x_u2+m_u4*x_u4);
y_m=1/(m_l3+m_u1+m_u2+m_u4)*(m_l3*y_l3+m_u1*y_u1+m_u2*y_u2+m_u4*y_u4);

figure(10)
pause(0.25)
% clf
hold on,axis equal
plot(x_l1,y_l1,'o')
plot([x_l1 x_l2],[y_l1 y_l2],'r', 'LineWidth',1)
plot(x_l2,y_l2,'o')
plot([x_l2 x_l3],[y_l2 y_l3],'r', 'LineWidth',1)
plot(x_l3,y_l3,'o')
plot([x_l3 x_u1],[y_l3 y_u1],'b', 'LineWidth',1)
plot(x_u1,y_u1,'o')
plot([x_u1 x_u2],[y_u1 y_u2],'b', 'LineWidth',1)
plot(x_u2,y_u2,'o')
plot([x_u2 x_u3],[y_u2 y_u3],'b', 'LineWidth',1)
plot(x_u3,y_u3,'o')
plot([x_u3 x_u4],[y_u3 y_u4],'b', 'LineWidth',1)
plot(x_u4,y_u4,'o')
plot([x_u4 x_u5],[y_u4 y_u5],'b', 'LineWidth',1)
plot(x_u5,y_u5,'o')
plot(x_m,y_m,'o','color','black')
   
x_m_save(i)=x_m;
error_save_x(:,i)=e;
% tau_save_x(:,i)=[tau_1x;tau_2x;tau_3x;tau_4x];
end

figure(100),hold on,plot(error_save_x')