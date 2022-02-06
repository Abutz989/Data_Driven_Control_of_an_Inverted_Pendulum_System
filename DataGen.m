% This script uses to ganarte iLQR controller data
% Defining the system and the parameters
global Mpend g l Ipend c kt ke R f Jtot
Mwheel=241e-3; Mrod=130e-3; Mmotor=75e-3; Mpend=Mwheel+Mmotor+Mrod/2;Iextra= 0.0015;
l=0.2566; Ipend=(Mwheel+Mmotor+Mrod/4)*l^2+Iextra;
g=9.798; c=0.0012;
kt=(25.5e-3)/2; ke=60/(2*pi*374)/2;f=0.000136;R=1.2;
Jwheel=0.00064;Jmotor=92.5e-7;Jtot=Jwheel+Jmotor;

A_full=[0 1 0;...
    Mpend*g*l/Ipend, -c/Ipend, kt*ke/(R*Ipend)+f/Ipend;...
    -Mpend*g*l/Ipend, c/Ipend -(kt*ke/R+f)*((Ipend+Jtot)/(Ipend*Jtot))];
B_full=[0; -kt/(R*Ipend); kt/R*((Ipend+Jtot)/(Ipend*Jtot))];

% Q&R are optimiztion parameters
Q_T=[1 0 0; 0 0.2 0; 0 0 1e-5];
R_T=0.01;
% real system parameters
GyroRes = 0.018; %gyro resulotion
noise_sigma=[0 0.01 0.01]';
StopTime=5;
Ts=0.001;
t=[0:Ts:StopTime-Ts];Jump=length(t);
BatchSize=500; % determine the number of steps that we predict       
angle = [-8:0.5:8]*pi/180; % set al the initial positions for the training data
% initiliazie logs
x_t=zeros(3,Jump*length(angle));
U=zeros(1,Jump*length(angle));

for k1=1:length(angle)
    x0=[angle(k1) 0 0]'; Uilqr=zeros(size(t)); % initilaize senerio
    X_hat=zeros(3,1); xt=x0;  %equlibrium point, and the actual state 
    u_hat=0;ut=0;

    for step=1:1:Jump   
        [Atag,Btag] = GetABTag(X_hat,u_hat,Ts);   % culculate equl' matrix around x_hat and u_hat
        P_t1=Q_T;
            for Kculc=BatchSize:-1:1                % discrete LQR for optimize path
                    Ktemp=-1*inv(R_T+Btag'*P_t1*Btag)*Btag'*P_t1*Atag;
                    Pt=Q_T+Atag*P_t1*Atag+Atag*P_t1*Btag*Ktemp;
                    P_t1=Pt;
                    Kvec2(Kculc,:)=Ktemp;
            end

         Uilqr(step)=Kvec2(1,:)*(xt-X_hat)+u_hat; % the current  control input
         [ode_time,ode_state] =  ode45(@(ode_time,ode_state) dynamic(ode_time,ode_state),[0 Ts],[(xt);Uilqr(step)]);  % culculte the next state in time t+dt.
         X_hat=xt;
         ut=Uilqr(step);
         u_hat=ut;
         xt=ode_state(end,1:3)';                                
    end

    %% simulate our results
    state = x0;
    states = zeros(3,length(t));
    states(:,1) = state;
    for i=1:Jump
      force = Uilqr(i);
      [Model_time,Model_state] =  ode45(@(Model_time,Model_state) dynamic(Model_time,Model_state),[0 Ts],[state; force]);
      state = Model_state(length(Model_state),1:3)';
    %   state(2) = quant(state(2),GyroRes);
      states(:,i) = state;
    end

    x_t(:,(1+(k1-1)*Jump):(Jump+((k1-1)*Jump)))=states(:,1:(end));
    % x_t_n(:,(1+(k1-1)*Jump):(Jump+((k1-1)*Jump)))=states(:,1:(end))+randn(3,5000).*noise_sigma;
    U(:,(1+(k1-1)*Jump):(Jump+((k1-1)*Jump)))=Uilqr(:,1:(end));
end    
%%  
figure(2)
subplot (2,1,1)
hold on
plot(t,states(1,:),'-k','LineWidth',2)
grid on
legend('MPC')
ylabel('Angle [rad]')
xlabel('Time [sec]')

subplot (2,1,2)
plot(t,Uilqr,'-k','LineWidth',2)
hold on
plot([0 5],[12 12],'--b','LineWidth',1)
grid on
title('Control Effort');
ylabel('V [Volt]')
xlabel('Time [sec]')
legend('iLQR')

DataLearn=[x_t' U'];
save('DataForTraning','DataLearn')

%% Find A_k,B_k
function [Atag Btag] = GetABTag(X_hat,u_hat,Ts)
%Jacobian matrix    
    global Mpend g l Ipend c kt ke R f Jtot
    A_J=[0 1 0;...
    Mpend*g*l/Ipend*cos(X_hat(1)), -c/Ipend, kt*ke/(R*Ipend)+f/Ipend;...
    -Mpend*g*l/Ipend*cos(X_hat(1)), c/Ipend -(kt*ke/R+f)*((Ipend+Jtot)/(Ipend*Jtot))]; 
    B_J=[0; -kt/(R*Ipend); kt/R*((Ipend+Jtot)/(Ipend*Jtot))];
    
    ssys=ss(A_J,B_J,eye(3),0);
    dsys=c2d(ssys,Ts);
Atag = dsys.A;
Btag = dsys.B;
end

%% Dynamic
function dXdt = dynamic(t,state)
X=state(1:3); V=state(4);
global Mpend g l Ipend c kt ke R f Jtot
dXdt(1)=X(2);
dXdt(2)= Mpend*g*l/Ipend*sin(X(1))-c/Ipend*X(2)+(kt*ke/(R*Ipend)+f/Ipend)*X(3)+(-kt/(R*Ipend))*V;
dXdt(3)=-Mpend*g*l/Ipend*sin(X(1))+c/Ipend*X(2)+(-(kt*ke/R+f)*((Ipend+Jtot)/(Ipend*Jtot)))*X(3)+(kt/R*((Ipend+Jtot)/(Ipend*Jtot)))*V;
dXdt=[dXdt';0];
end
