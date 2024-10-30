% Code for the paper "Computational adaptive optimal control with an
% application to a car engine control problem",  Yu Jiang and Zhong-Ping
% Jiang,vol. 48, no. 10, pp. 2699-2704, Oct. 2012.
% Copyright 2011-2014 Yu Jiang, New York University.

function [K,K0, P, P0]=Jiang2012AutomaticaEditedDataFromSUMO(Vx, Dxx, XX, XU)
clc;
x_save=[];
t_save=[];
flag=1;  % 1: learning is on. 0: learning is off.

% System matrices used for simulation purpose
m = 1360;
Iz = 1993;
lf = 1.45;
lr = 1.06;
% lr = 1.45;
% lf = 1.06;
Cf = 1.51*100000;
Cr = 1.46*100000;

A = [
    0 1 0 0;
    0 -(1/(m*Vx))*(2*Cf+2*Cr) (2*Cf+2*Cr)/(m) -(2*Cf*lf-2*Cr*lr)/(m*Vx);
    0 0 0 1;
    0 -(1/(Iz*Vx))*(2*lf*Cf-2*lr*Cr) (2*Cf*lf-2*Cr*lr)/Iz -(1/(Iz*Vx))*(2*lf^2*Cf+2*lr^2*Cr)
    ];
B = [0 2*Cf/m 0 2*lf*Cf/Iz]';

[xn,un]=size(B);%size of B. un-column #, xn row #

% Set the weighting matrices for the cost function
Q = diag([20 50 2000 3000]); %% best
R = 1;

% Initialize the feedback gain matrix
p = [-1000;-0.5;-3;-18];
Kinit = place(A,B,p);
K = Kinit

% i1=(rand(1,100)-.5)*1000;
% ExpNoiseFreq = i1;
% save('ExpNoiseFreq2.mat', 'ExpNoiseFreq');


% Run the simulation and obtain the data matrices \delta_{xx}, I_{xx},
% and I_{xu}
[K0,P0]=lqr(A,B,Q,R) % Calculate the ideal solution for comparion purpose

Dxx=processing_Dxx(Dxx); % Only the distinct columns left


% K=zeros(un,xn);  % Initial stabilizing feedback gain matrix
P_old=zeros(xn);P=eye(xn)*10; % Initialize the previous cost matrix
K_old = K;
it=0;            % Counter for iterations
p_save=[];       % Track the cost matrices in all the iterations
p_saveOld = [];
k_save=[];       % Track the feedback gain matrix in each iterations
k_saveOld = [];


k_save=[norm(K-K0)];

while norm(P-P_old)>1e-5 & it<16   % Stopping criterion for learning
    it=it+1                         % Update and display the # of iters
    P_old=P;                        % Update the previous cost matrix
    QK=Q+K'*R*K;                    % Update the Qk matrix
    X2=XX*kron(eye(xn),K');         %
    X1=[Dxx,-X2-XU];                % Left-hand side of the key equation
    Y=-XX*QK(:);                    % Right-hand side of the key equation
    pp=X1\Y;                        % Solve the equations in the LS sense
    P=reshape_p(pp);                % Reconstruct the symmetric matrix
    p_save=[p_save,norm(P-P0)];     % Keep track of the cost matrix
    p_saveOld = [p_saveOld,norm(P-P_old)];
    BPv=pp(end-(xn*un-1):end);
    K=inv(R)*reshape(BPv,un,xn)/2   % Get the improved gain matrix
    k_save=[k_save,norm(K-K0)];     % Keep track of the control gains
end

% Plot the trajectories
figure(1)
plot([0:length(p_save)-1],p_save,'o',[0:length(p_save)-1],p_save)
%axis([-0.5,it-.5,-5,15])
legend('||P_k-P^*||')
xlabel('Number of iterations')

figure(2)
plot([0:length(k_save)-1],k_save,'^',[0:length(k_save)-1],k_save)
%axis([-0.5,it+0.5,-.5,2])
legend('||K_k-K^*||')
xlabel('Number of iterations')

figure
plot([0:length(p_saveOld)-1],p_saveOld,'o',[0:length(p_saveOld)-1],p_saveOld)
%axis([-0.5,it-.5,-5,15])
legend('||P_k-P_{old}||')
xlabel('Number of iterations')


% Post-learning simulation
% [tt,xx]=ode23(@(t,x) mysys(t, x, Vx), [t(end) 100], X(end,:)');

% Keep track of the post-learning trajectories
t_final=[t_save;tt];
x_final=[x_save;xx];

figure(3)
plot(t_final,x_final(:,1:4),'Linewidth',2)
%axis([0,10,-100,200])
legend('x_1','x_2','x_3','x_4')
xlabel('Time (sec)')

figure(4)
plot(t_final,sqrt(sum(x_final(:,1:4).^2,2)),'Linewidth',2)
%axis([0,200,-50,200])
legend('||x||')
xlabel('Time (sec)')

% figure(5)
% plot(t_final,3.6*x_final(:,1),'k-.', ...
%     t_final, x_final(:,4),'-','Linewidth',2)
% %axis([0,10,-80,50])
% legend('y_1 (MAF)','y_2 (MAP)')
% xlabel('Time (sec)')

% The following nested function gives the dynamics of the sytem. Also,
% integraters are included for the purpose of data collection.
    function dX=mysys(t,X, Vx)
        %global A B xn un i1 i2 K flag
        % System matrices used for simulation purpose
        m = 1360;
        Iz = 1993;
        lf = 1.45;
        lr = 1.06;
        % lr = 1.45;
        % lf = 1.06;
        Cf = 1.51*100000;
        Cr = 1.46*100000;
        A = [
            0 1 0 0;
            0 -(1/(m*(Vx)))*(2*Cf+2*Cr) (2*Cf+2*Cr)/(m) -(2*Cf*lf-2*Cr*lr)/(m*(Vx));
            0 0 0 1;
            0 -(1/(Iz*(Vx)))*(2*lf*Cf-2*lr*Cr) (2*Cf*lf-2*Cr*lr)/Iz -(1/(Iz*(Vx)))*(2*lf^2*Cf+2*lr^2*Cr)
            ];
        B = [0 2*Cf/m 0 2*lf*Cf/Iz]';
        x=X(1:xn);
        
        if t>=2;   % See if learning is stopped
            flag=0;
        end

        if flag==1
            u=zeros(un,1);
            for i=i1
                u(1)=u(1)+sin(i*t)/length(i1); % constructing the
                % exploration noise
            end
            u=-K*x+100*u;
        else
            u=-K*x;
        end

        dx=A*x+B*u;
        dxx=kron(x',x')';
        dux=kron(x',u')';
        dX=[dx;dxx;dux];
    end

% This nested function reconstruct the P matrix from its distinct elements
    function P=reshape_p(p)
        P=zeros(xn);
        ij=0;
        for i=1:xn
            for j=1:i
                ij=ij+1;
                P(i,j)=p(ij);
                P(j,i)=P(i,j);
            end
        end
    end

% The following nested function removes the repeated columns from Dxx
    function Dxx=processing_Dxx(Dxx)
        ij=[]; ii=[];

        for i=1:xn
            ii=[ii (i-1)*xn+i];
        end

        for i=1:xn-1
            for j=i+1:xn
                ij=[ij (i-1)*xn+j];
            end
        end

        Dxx(:,ii)=Dxx(:,ii)/2;
        Dxx(:,ij)=[];
        Dxx=Dxx*2;
    end
end