%% forward kinematic
clear all
m1=1;
m2=2;
l1=0.5;
l2=0.5;

q1=120/180*pi;
q2=60/180*pi;
q3=310/180*pi;
q4=340/180*pi;
x_1=0;
y_1=0;
x_2=x_1+l1*cos(q1);
y_2=y_1+l1*sin(q1);
x_3=x_2+l2*cos(q2);
y_3=y_2+l2*sin(q2);
x_4=x_3+l2*cos(q3);
y_4=y_3+l2*sin(q3);
x_5=x_4+l1*cos(q4);
y_5=y_4+l1*sin(q4);
dt=1/20;
for i=1:1/dt

J=[-l1*sin(q1) -l2*sin(q2) 0 0;...
   -l1*sin(q1) -l2*sin(q2) -l2*sin(q3) -l1*sin(q4);...
   l1*cos(q1) l2*cos(q2) l2*cos(q3) l1*cos(q4)];

x_r5=0.5-i*dt;  
y_r5=0.2*sin(i*dt*pi);

e=[x_3-0,x_5-x_r5,y_5-y_r5]';
d_q=-5*pinv(J)*e;
d_q1=d_q(1);d_q2=d_q(2);
d_q3=d_q(3);d_q4=d_q(4);

q1=q1+dt*d_q1;
q2=q2+dt*d_q2;
q3=q3+dt*d_q3;
q4=q4+dt*d_q4;

x_2=x_1+l1*cos(q1);
y_2=y_1+l1*sin(q1);
x_3=x_2+l2*cos(q2);
y_3=y_2+l2*sin(q2);
x_4=x_3+l2*cos(q3);
y_4=y_3+l2*sin(q3);
x_5=x_4+l1*cos(q4);
y_5=y_4+l1*sin(q4);

x_m=1/(4*m1)*(m1*x_1+m2*x_3+m1*x_5);
y_m=1/(4*m1)*(m1*y_1+m2*y_3+m1*y_5);

g=10;
tau_1x=(x_3-x_1)*m2*g+(x_5-x_1)*m1*g;
tau_2x=(x_3-x_2)*m2*g+(x_5-x_2)*m1*g;
tau_3x=(x_5-x_3)*m1*g;
tau_4x=(x_5-x_4)*m1*g;

figure(10)
pause(0.25)
% clf
hold on,axis equal
plot(x_1,y_1,'o')
plot([x_1 x_2],[y_1 y_2],'r', 'LineWidth',1)
plot(x_2,y_2,'o')
plot([x_2 x_3],[y_2 y_3],'r', 'LineWidth',1)
plot(x_3,y_3,'o')
plot([x_3 x_4],[y_3 y_4],'b', 'LineWidth',1)
plot(x_4,y_4,'o')
plot([x_4 x_5],[y_4 y_5],'b', 'LineWidth',1)
plot(x_5,y_5,'o')
plot(x_m,y_m,'o','color','black')
   
x_m_save(i)=x_m;
error_save_x(:,i)=e;
tau_save_x(:,i)=[tau_1x;tau_2x;tau_3x;tau_4x];
end

figure(100),hold on,plot(error_save_x')
figure(200),hold on,plot(tau_save_x')