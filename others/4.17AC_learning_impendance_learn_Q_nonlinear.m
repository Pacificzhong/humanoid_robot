clear all
% close all
%clc
%%% System description:
dim_A=4;
dim_B=1;
dt=0.01;
M=1;
m=1;
l=0.5;
g=10;
A=[0 1 0 0;...
    -1/(M*l^2)*l*M*g 0 1/(M*l^2)*l*M*g 0;...
    0 0 0 1;...
    1/(m*l^2)*l*m*g 0 1/(m*l^2)*l*m*g 0];
B=[0 0 0 1/(M*l^2)]';

A_c=eye(4)+dt*A;
B_c=dt*B;

Q_1=eye(4);
R=0.01*eye(1);

T=dt:dt:20;
[P,K,G] = dare(A_c,B_c,Q_1,R);
K_l2=-G
%%% initialization:
Num_theta=sum(1:dim_A+dim_B);
theta_i=zeros(Num_theta,1);
V_save=[];
K_x_V=[];
K_error=[];
theta_save=[];
X_k_save=[];
K_l=K_l2*0.8-0.1;
K_l_AC=K_l;

%%% learning parameters
Num_ep=10;
noise_trial=50;
gamma=1;
P_i_level=1e3;

for j=1:Num_ep
    
    P_i=eye(Num_theta,Num_theta)*P_i_level;
    X_k0=[0.5 0 0 0]';
    X_k_V=X_k0;
    alpha=X_k_V(1);
    beta=X_k_V(3);
    d_alpha=0;
    d_beta=0;
    
    for i=1:length(T)
        X_k_save=[X_k_save X_k_V];
        K_x_V=[K_x_V; K_l];
        X_k_V_=X_k_V;
        mu_V=K_l*X_k_V_+(Num_ep+1-j)/Num_ep*noise_trial* randn; %
        dd_alpha=1/(M*l^2)*( l*cos(X_k_V(1)+X_k_V(3))*m*g+l*(m+M)*g*cos(X_k_V(1))  );
        dd_beta=1/(m*l^2)*( l*cos(X_k_V(3)+X_k_V(1))*m*g +mu_V);
        
        d_alpha=d_alpha+dt*dd_alpha;
        d_beta=d_beta+dt*dd_beta;
        
        alpha=alpha+dt*d_alpha;
        beta=beta+dt*d_beta;
        X_k_V=A_c*X_k_V_+B_c*mu_V;
        if abs(X_k_V(1))>1
            X_k_V(1)=sign(X_k_V(1))*1;
        end
        if abs(X_k_V(3))>1
            X_k_V(3)=sign(X_k_V(3))*1;
        end
        
        if mod(i,10)==0 && 1==1
            alpha=X_k_V(1)+90/180*pi;
            beta=X_k_V(3)+180/180*pi;
            figure(20),
            axis([-0.5 0.5 -0.5 0.5]);
            pause(0.01)
            clf
            hold on,axis equal
            plot(0,0,'o')
            plot([0 l*cos(alpha)],[0 l*sin(alpha)],'r', 'LineWidth',1)
            plot([l*cos(alpha) l*cos(beta+alpha)+l*cos(alpha)],[l*sin(alpha) l*sin(alpha)+l*sin(beta+alpha)],'r', 'LineWidth',1)
        end
        
        if i>1
            c_k=X_k_V_'*Q_1*X_k_V_ + mu_V'*R*mu_V;%
            phi_e=gradient_Q(X_k_V_,X_k_V,mu_V,K_l,gamma);
            
            gradient=P_i*phi_e*(c_k-phi_e'*theta_i)/(1+phi_e'*P_i*phi_e);
            P_i=P_i-P_i*phi_e*phi_e'*P_i/(1+phi_e'*P_i*phi_e);
            theta_i=theta_i+gradient;
            
            [H_21,H_22]=H_2_theta(X_k_V_,mu_V,theta_i); % reshape the theta to H,
            
            index_Jum=0.1/(1+norm(gradient)*0.05);%  当critic梯度太大时，actor梯度变小
            J_sum=index_Jum*(H_21+K_l_AC*H_22);%*(X_k_V_*X_k_V_')
            
            K_l_AC=K_l_AC-J_sum; %
            update_step=10;
            if i>update_step*1 && mod(i,update_step)==0
                K_l=K_l_AC;
            end
            
            V_save=[V_save; (c_k).^2];
            mu_save(:,i)=mu_V;
            theta_save=[theta_save; [norm(gradient)*0.05 norm(J_sum)]];
            
        end
    end
%         K_l=-pinv(H_22)*H_21;
end

K_l

Time_t=[1:length(X_k_save)]*dt;
figure(19),hold on,plot(Time_t,X_k_save(1,:),'linewidth',1)
figure(19),hold on,plot(Time_t,X_k_save(3,:),'linewidth',1)
figure(21),hold on,plot(Time_t,K_x_V,'linewidth',1),
K_error=K_x_V-kron(K_l2,ones(length(K_x_V),1));
figure(22),hold on,plot(Time_t,abs( K_error(:,1))+abs( K_error(:,2))+abs( K_error(:,3)),'linewidth',1),

function [H_21,H_22]=H_2_theta(X_k_V_,mu_V,theta_i)
L_num=length(X_k_V_)+length(mu_V);
theta_i_new=theta_i;
N_num=0;
for i=1:L_num-1
    N_num=N_num+L_num+1-i;
    H_21(1,i)=theta_i_new(N_num)/2;
end
H_22=[theta_i_new(end)];
end

function phi_e=gradient_Q(X_k_V_,X_k_V,mu_V,K_l,gamma)
H_xx_=[X_k_V_;mu_V];
H_xx=[X_k_V;K_l*X_k_V];
L_num=length(H_xx_);
phi_e_all=[kron(H_xx_,H_xx_)-gamma*kron(H_xx,H_xx)];
Num_Q=1;
for ii=1:L_num
    for jj=1:L_num
        if jj>=ii
            phi_e(Num_Q,1)=phi_e_all((ii-1)*L_num+jj);
            Num_Q=Num_Q+1;
        end
    end
end

end