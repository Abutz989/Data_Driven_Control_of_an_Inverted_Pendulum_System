function [dummy] =  My_Pendulum_sin_local()
% Region of attraction analysis on the pendulum system with NN controller
% credit : https://github.com/heyinUCB/Stability-Analysis-using-Quadratic-Constraints-for-Systems-with-Neural-Network-Controllers
% Modified from He Yin work and his paper :
% H. Yin, P. Seiler and M. Arcak, "Stability Analysis using Quadratic Constraints for Systems with Neural Network Controllers," in IEEE Transactions on Automatic Control, doi: 10.1109/TAC.2021.3069388.
%% parameters
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

%% x^+ = AG*x + BG1*q + BG2*u
AG = [sysd.A ,zeros(3,9);...
    eye(3),zeros(3,9);...
    zeros(3), eye(3),zeros(3,6);...
    zeros(3,6), eye(3),zeros(3)];
% describes how q enters the system
BG1 =  zeros(12,1);
% describes how u enters the system
BG2 = [sysd.B; zeros(9,1)];
  
nG = size(AG, 1);
nu = 1;
nq = 1;

%  v_Delta = CG*xG + [DG1 DG2]*[q; u] = xG
CG =  zeros(1,12);
CG(1) = 1;
DG1 = 0;
DG2 = 0;

%% load weights and biases of the NN controller     

load('TrainedNet.mat')

n1 = size(W1,1);
n2 = size(W2,1);
n3 = size(W3,1);
nphi = n1+n2+n3;

b1 = W1_b;
b2 = W2_b;
b3 = W3_b;

LReluAlpha1 = 0.45;
LReluAlpha2 = 0.35;
%% bounds for the inputs to the nonlinearity
xeq = zeros(12,1);
v1eq = W1*xeq + b1;
w1eq = LeakyReLU(v1eq,LReluAlpha1);
v2eq = W2*w1eq + b2;
w2eq = LeakyReLU(v2eq,LReluAlpha2);
v3eq = W3*w2eq + b3; % This is also u_*
% usat = sat(v3) = sat(u)
deltav1 = 0.5 ;
v1up = v1eq + deltav1 + b1 ;
v1lb = v1eq - deltav1 + b1;

% 
alpha1 = min((LeakyReLU(v1up,LReluAlpha1)-LeakyReLU(v1eq,LReluAlpha1))./(v1up-v1eq), (LeakyReLU(v1eq,LReluAlpha1)-LeakyReLU(v1lb,LReluAlpha1))./(v1eq-v1lb));
beta = 1;
w1up = LeakyReLU(v1up,LReluAlpha1);
w1lb = LeakyReLU(v1lb,LReluAlpha1);

v2up = W2*1/2*(w1up+w1lb) + b2 + abs(W2)*1/2*abs(w1up-w1lb);
v2lb = W2*1/2*(w1up+w1lb) + b2 - abs(W2)*1/2*abs(w1up-w1lb);
alpha2 = min((LeakyReLU(v2up,LReluAlpha2)-LeakyReLU(v2eq,LReluAlpha2))./(v2up-v2eq), (LeakyReLU(v2eq,LReluAlpha2)-LeakyReLU(v2lb,LReluAlpha2))./(v2eq-v2lb));

w2up = LeakyReLU(v2up,LReluAlpha2);
w2lb = LeakyReLU(v2lb,LReluAlpha2);
v3up = W3*1/2*(w2up+w2lb) + b3 + abs(W3)*1/2*abs(w2up-w2lb);
v3lb = W3*1/2*(w2up+w2lb) + b3 - abs(W3)*1/2*abs(w2up-w2lb);
umax = 10;
alpha3 = min((sat(v3up,umax,-umax)-sat(v3eq,umax,-umax))./(v3up-v3eq), (sat(v3eq,umax,-umax)-sat(v3lb,umax,-umax))./(v3eq-v3lb));

Alpha = blkdiag(diag(alpha1),diag(alpha2),alpha3);
Beta = beta*eye(nphi);

% Filter for sector IQC
Psi_phi = [Beta, -eye(nphi);...
          -Alpha, eye(nphi)];

%% Define IQCs for the nonlinearity x1 - sin(x1)
% x1 - sin(x1) is slope-restricted in [0, 2] globally ---> off-by-one IQC
% x1 - sin(x1) is slope-restricted in [0, 0.1224] locally n x1 in [-0.5, 0.5] ---> off-by-one IQC

% x1 - sin(x1) is sector-bounded in [0, 1.22] globally ---> sector IQC
% x1 - sin(x1) is sector-bounded in [0, 1] locally on x1 in [-pi,pi]
% x1 - sin(x1) is sector-bounded in [0, 0.7606] locally on x1 in [-2.5,2.5]
% x1 - sin(x1) is sector-bounded in [0, 0.0411] locally on x1 in [-0.5,0.5]
% define the filter for off-by-one IQC
x1bound = 7/180*pi;
L_slope = 1 - cos(x1bound);
m_slope = 0;
Apsi = zeros(1);
npsi = size(Apsi,1);
Apsi = Apsi*dt + eye(npsi);
Bpsi1 = -L_slope;
Bpsi2 = 1;
Bpsi1 = Bpsi1*dt;
Bpsi2 = Bpsi2*dt;
Cpsi = [1;0];
Dpsi1 = [L_slope; -m_slope];
Dpsi2 = [-1; 1];
% M matrix for off-by-one IQC         
M_off = [0, 1;...
         1, 0];
     
% define the filter for sector IQC
L_sector = (x1bound - sin(x1bound))/x1bound;
m_sector = 0;
Psi_sec = [L_sector, -1;...
           -m_sector,    1];
% M matrix for sector IQC
M_sec = [0, 1;...
         1, 0];
     
%% construct the extended system
Abar = [AG, zeros(nG,npsi);...
        Bpsi1*CG, Apsi];
Bbar = [BG1, BG2;...
        Bpsi1*DG1+Bpsi2, Bpsi1*DG2];
Cbar = [Dpsi1*CG, Cpsi];
Dbar = [Dpsi1*DG1+Dpsi2, Dpsi1*DG2];
nzeta = nG + npsi;

Rzeta = [eye(nzeta), zeros(nzeta, nphi+nq)];
Rq = [zeros(nq,nzeta+nphi), eye(nq)];
Rusat = [zeros(nu, nzeta+n1+n2), eye(nu), zeros(nu,nq)];
Rwphi = [zeros(nphi,nzeta), eye(nphi), zeros(nphi,nq)];
Rvphi = [blkdiag([W1,zeros(n1,npsi)], W2, [W3,zeros(n3,nu)]),zeros(nphi,nq)];
Rp = [1, zeros(1, nzeta-1+nphi+nq)];

%% Convex Optimization - compute ROA
cvx_begin sdp quiet
    cvx_solver sedumi
    % Variables
    variable P(nzeta,nzeta) symmetric;
    variable Tphi(nphi,nphi) diagonal; 
    variable lambda1;
    variable lambda2;
    lambda1 >= 0;
    lambda2 >= 0;
    Tphi(:) >= 0;
    P >= 1e-8*eye(nzeta);

    % Term to bound nonlinearity Phi
    Mphi = [zeros(nphi), Tphi; Tphi, zeros(nphi)];    
    Rphi = [Rvphi; Rwphi];
    Qphi = Rphi'*Psi_phi'*Mphi*Psi_phi*Rphi;
    
    % Term to represent V(k+1) - V(k)
    S = [Abar'*P*Abar - P, Abar'*P*Bbar;...
         Bbar'*P*Abar, Bbar'*P*Bbar];
    RV = [Rzeta; Rq; Rusat];
    QV = RV'*S*RV;
    
    % Term that uses off-by-one IQC to bound the nonlinearity Delta
    Qoff = lambda2*RV'*[Cbar Dbar]'*M_off*[Cbar Dbar]*RV;
    
    % Term that uses sector IQC to bound the nonlinearity Delta
    R_sec = [Rp;...
             Rq];
    Qsec = lambda1*R_sec'*Psi_sec'*M_sec*Psi_sec*R_sec;
    
    % Matrix Inequality
    QV + Qphi + Qsec + Qoff <= -1e-10*eye(nzeta+nphi+nq);
    for i = 1:n1
    % enforce {x: x'Px<=1} \subset {x: |[W0(i,:) 0]*x| <= r1} 
        [deltav1^2+norm(b1,1), [W1(i,:) zeros(1,npsi)];...
         [W1(i,:) zeros(1,npsi)]', P] >= 0;...
    end
    % enforce {x: x'Px<=1} \subset {x: |x1| <= x1bound}
    [x1bound^2, [1,zeros(1,11), zeros(1,npsi)];...
     [1,zeros(1,11), zeros(1,npsi)]', P] >=0;...
    minimize(trace(P(1:nG,1:nG)))
cvx_end
P
if ~isnan(P)
    radii = 1./sqrt(eig(P(1:nG,1:nG)));
    traceP = trace(P(1:nG,1:nG))
end

%% plot results
% Simulation results
figure()
N1 = 45;
x1box = linspace(-0.2,0.2,N1);
N2 = 45;
x2box = linspace(-1,1,N2);
xIC = [x1box x1box  x1box(1)*ones(1,N1) x1box(end)*ones(1,N1); ...
    x2box(1)*ones(1,N2) x2box(end)*ones(1,N2) x2box x2box];
Nstep = 1500;
for i=1:size(xIC,2)
    x0 = [xIC(:,i); zeros(1,10)'];
    [x,u] = nnclosedloop(Nstep,x0,{W1,W2,W3},{b1,b2,b3},umax);
    if abs(x(1,end)) <= 0.2
        plot(x(1,:),x(2,:),'g');
    else
        plot(x(1,:),x(2,:),'r');
    end
    hold on
end

%% Hyperplanes 
x1 = linspace(-2,2,1e3);

% for i=1:n1
%     plot(x1,(deltav1 - W1(i,1)*x1 )/(W1(i,2)) ,'color',mycolor('orange')) 
%     plot(x1,(-deltav1 - W1(i,1)*x1)/(W1(i,2)),'color',mycolor('orange'))
%     hold on;
% end
plot([x1bound,x1bound],[-9,9],'color','k')
plot([-x1bound,-x1bound],[-9,9],'color','k')

% ROA
pvar x1 x2
V = [x1,x2]*[eye(2) zeros(2,10)]*P(1:nG,1:nG)*[eye(2);zeros(10,2)]*[x1;x2];
domain1 = [-10, 10, -10, 10];
[C,h] = pcontour(V,1,domain1,'r',[300, 300]);
h.LineColor = mycolor('coolblue');
h.LineWidth = 4;
hold on

% xeq
plot(xeq(1),xeq(2),'kx','MarkerSize',10);
hold off;


grid on;
axis([-0.35 0.35 -2 2]);
xlabel('$\alpha$','interpreter','latex')
ylabel('$\dot{\alpha}$','interpreter','latex')

garyfyFigure
end