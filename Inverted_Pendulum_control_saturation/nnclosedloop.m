function [x,u] = nnclosedloop(Nstep,x0,W,b,umax)
% Simulate system for N steps from the initial condition x0.
%%
Mwheel=241e-3; Mrod=130e-3; Mmotor=75e-3; Mpend=Mwheel+Mmotor+Mrod/2;Iextra= 0.0015;
l=0.2566; Ipend=(Mwheel+Mmotor+Mrod/4)*l^2+Iextra;
g=9.798; c=0.0012;
kt=(25.5e-3)/2; ke=60/(2*pi*374)/2;f=0.000136;R=1.2;
Jwheel=0.00064;Jmotor=92.5e-7;Jtot=Jwheel+Jmotor;

A=[0 1 0;...
    Mpend*g*l/Ipend, -c/Ipend, kt*ke/(R*Ipend)+f/Ipend;...
    -Mpend*g*l/Ipend, c/Ipend -(kt*ke/R+f)*((Ipend+Jtot)/(Ipend*Jtot))];
B=[0; -kt/(R*Ipend); kt/R*((Ipend+Jtot)/(Ipend*Jtot))];
dt = 0.001; 

sys = ss(A,B,eye(3),0);
sysd = c2d(sys,dt);
AG = [sysd.A ,zeros(3,9);...
    eye(3),zeros(3,9);...
    zeros(3), eye(3),zeros(3,6);...
    zeros(3,6), eye(3),zeros(3)];
% describes how q enters the system
BG2 = [sysd.B; zeros(9,1)];
  
%%

% Initialize outputs
u0 = LOCALnn(x0,W,b,umax);
Nu = numel(u0);
Nx = numel(x0);

x = zeros(Nx,Nstep);
x(:,1) = x0;

u = zeros(Nu,Nstep);
u(:,1) = u0;

% Simulate System

for i=2:Nstep

    x(:,i) = AG*x(:,i-1)+BG2* u(:,i-1);
    u(:,i) = LOCALnn(x(:,i),W,b,umax);
end


%%
% LOCAL Function to evaluate the Neural Network
function u = LOCALnn(x,W,b,umax)
    
Nlayer = numel(W);
z = x;
for i=1:(Nlayer-1)
    z = W{i}*z + b{i};
    
    % XXX - Nonlinearity could be input as a function handle
    if i == 1
        z = LeakyReLU(z,0.45);
    else
        z = LeakyReLU(z,0.35);
    end
end
u = W{end}*z+b{end};
u = sat(u,umax,-umax);
    

