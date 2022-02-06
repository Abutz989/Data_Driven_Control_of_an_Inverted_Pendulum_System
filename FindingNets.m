clear
disp(['****** Night Run******'])
Mwheel=241e-3; Mrod=130e-3; Mmotor=75e-3; Mpend=Mwheel+Mmotor+Mrod/2;Iextra= 0.0014;
l=0.2566; Ipend=(Mwheel+Mmotor+Mrod/4)*l^2+Iextra;
g=9.798; c=0.0012;
kt=(25.5e-3)/2; ke=60/(2*pi*374)/2;f=0.000136;R=1.2;
Jwheel=0.00064;Jmotor=92.5e-7;Jtot=Jwheel+Jmotor;


seed =-1;
options = trainingOptions('adam', ...
    'MaxEpochs',600, ...
    'InitialLearnRate',0.1, ...
    'LearnRateSchedule','piecewise', ...
    'LearnRateDropFactor',0.9, ...
    'LearnRateDropPeriod',400, ...
    'Verbose',0, ...
    'Plots','none');

load('DataForTraning.mat')
XTrain = [DataLearn(4:end-1,1:3),DataLearn(3:end-2,1:3),DataLearn(2:end-3,1:3),DataLearn(1:end-4,1:3)];
YTrain = DataLearn(5:end,4);

alphaInt = 6*pi/180; 
StopTime=10;
Ts=0.001;
time=[0.000:Ts:StopTime];
states = zeros(3,length(time));

for seed=1:300 
    rng(seed);
    W1 = randn(15,12);
    W1_b = randn(15,1);
    W2 = randn(6,15);
    W2_b = randn(6,1);
    W3 = randn(1,6);
    W3_b = 0.5;
 
    layers = [
    sequenceInputLayer(12,"Name","Xk_1:4")
    flattenLayer("Name","flatten")
    fullyConnectedLayer(15,"Name","Layer1","Bias",W1_b,"Weights",W1)
    leakyReluLayer(0.45,"Name","leaky1")
    fullyConnectedLayer(6,"Name","Layer2","Bias",W2_b,"Weights",W2)
    leakyReluLayer(0.3,"Name","leaky2")
    fullyConnectedLayer(1,"Name","outLayer","Bias",W3_b,"Weights",W3)
    regressionLayer("Name","V_k1")];


    net = trainNetwork(XTrain',YTrain',layers,options);
    W1=double(net.Layers(3).Weights);
    W1_b=double(net.Layers(3).Bias);
    W2=double(net.Layers(5).Weights);
    W2_b=double(net.Layers(5).Bias);
    W3=double(net.Layers(7).Weights);
    W3_b=double(net.Layers(7).Bias);

    out = sim ('LabSim.slx');
    pause(1);
    alpha =out.States.Data(:,1);
    s_u = out.States.Data(:,3);

    if norm(alpha)<1.5
       clc
       disp(['seed = ', num2str(seed),' norm = ',num2str(norm(alpha))])
       save(['Nets/Net_for_seed_',num2str(seed),'with_norm_',num2str(norm(alpha)),'.mat'],'net','seed');
    end
end
