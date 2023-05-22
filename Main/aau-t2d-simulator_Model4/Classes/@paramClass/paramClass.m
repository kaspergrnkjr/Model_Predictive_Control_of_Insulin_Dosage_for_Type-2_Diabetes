%This script defines a class for the parameters of a T2D patient
%The properties of this class are structures for each model
%part/compartment of the simulator. The object have the defult values that
%was used in the paper: 
classdef paramClass
   properties
    
    %Glucose submodel:
    VGBC=3.5; VGBF=4.5; VGH=13.8; VGL=25.1;
    VGG=11.2; VGK=6.6; VGPC=10.4; VGPF=67.4; 
    QGB=5.9; QGH=43.7; QGA=2.5; QGL=12.6;
    QGG=10.1; QGK=10.1; QGP=15.1;
    TGB=2.1; TGP=5.0;

    %Diffusion term on GH:
    sig_diff = 2*sqrt(1/(24*60))/0.056;
    
    %Insulin submodel: 
    VIB=0.26; VIH=0.99; VIG=0.94; VIL=1.14; VIK=0.51; 
    VIPF=6.74;
    QIB=0.45; QIH=3.12; QIA=0.18;
    QIK=0.72; QIP=1.05; QIG=0.72; TIP=20.0; mpan0=6.33;
    QIL=0.9; VIPC=0.74;
    
    %Glucagon: 
    VGamma=6.74;
    
    %Glucose absorption parameters:
    fg=0.9; Kq1=0.68; Kq2=0.00236; k12=0.08; kmin=0.005; kmax=0.05;...
    kabs=0.08;
    
    k12GIH=0.037; kabsGIH=0.037;
    k12GIM=0.021; kabsGIM=0.021;
    k12GIL=0.012; kabsGIL=0.012;
    k12GIvL=0.006; kabsGIvL=0.006;

    %Glucose metabolic rates:
    c1 =0.067 %cIPGU;
    c2 = 1.59; %cIHGPinft;
    c3 = 0.62; %cGHGP;
    c4 = 1.72; %cIHGUinft;
    c5 = 2.03 %cGHGU;
    d1=1.126; %dIPGU;
    d2=0.683; %dIHGPinft;
    d3=0.14; %dGHGP;
    d4=0.023; %dIHGUinft;
    d5=1.59; %dGHGU
    SHGU =1; 
    SHGP = 1; 
    SPGU = 1; 
    
    %Pancreas model:
    zeta1=0.0026; zeta2=0.000099;
    ml0 = 6.33;
    Kl = 0.0572;
    %kdmdpan=0.0572*6.33; ml0*Kl
    Ks=0.0572; %Kpan
    gammapan=2.366; alphapan=0.615;
    betapan=0.931; N1=0.0499; N2=0.00015; 
    KILLPAN = 0;
    Sfactor = 1; 
    %GLP-1:
    VPSI=11.31; Kout=68.30411374407583; CF2=21.151177251184837;
    tpsi=35.1; zeta=8.248; %kpsi
    RmaxC=5;
    
    %GLP-1 agonists:
    %Exenatide: 
    ah2  = 0.013327726243785; bh2 = 0.013327635106873;
    ch2 = 1.132015661298425*1000; 
    
    %Semaglutide:
    ah24 = 6.542156013872301e-05; bh24 = 0.002630843797799; 
    ch24 = 3.459871138462720e+08/1000; 

    %vildagliptin Fv=77.2%
    Fv=0.772;
    ka1=1.26/60; ka2=1.05/60; CL=36.4/60; CLic=40.1/60; Vp=97.3;
    Vc=22.2; kvd = 71.9; %kdvil=71.9;
    k2v=23.4/60; %k2vil 
    koff=0.612/60; RmaxP=13; kdeg=0.110/60;
    
    %Metformin:
    kgo=1.88e-03; kgg=1.85e-03; kpg=4.13; kgl=0.46; kpl=1.01e-02;
    klp=0.91; kpo=0.51; vGWmax=0.9720; vLmax=0.7560; vPmax=0.2960;
    nGW=2; nL=5; nP=5; phiGW50=431;
    phiL50=521; phiP50=1024; rhoalpha=2.70e+04/500000; rhobeta=2.70e+04/500000;
    alpham=0.06; betam=0.1;
    

% %P = 0.013691570387547
% %    0.015050127414793
% %    0.035660745579443
% %    6.740000000000000
% %    0.040585233868227
% %    0.085809546448744
% %    0.026892564613936
% %   15.000000000000000
% %P = [pfa,qfa,bfa,VIPF,kcl,rfa,kla,Cmax];
%     %Long-acting insulin: %Default: Insulin glargin 
% 
    pla=0.014023809879501; rla=0.005642135109700; qla=0.007287049037943;...
    bla=0.088371175275079;...
    Cmax=15; kla=0.033904763958221; kcll =0.005347967285141;

%  P=  0.014023809879501
%    0.007287049037943
%    0.088371175275079
%    6.740000000000000
%    0.005347967285141
%    0.005642135109700
%    0.033904763958221
%   15.000000000000000
%P = [pfa,qfa,bfa,VIPF,kcl,rfa,kla,Cmax];
%     %Long-acting insulin: %Default: Insulin glargin 
% 
%     pla=0.014023809879501; rla=0.005642135109700; qla= 0.007287049037943; bla=0.088371175275079;...
%     Cmax=15; kla=0.033904763958221; kcll =0.005347967285141;



%     P = [0.017835764888995;
%    0.000030201564446;
%    0.104283036515623;
%    6.740000000000000;
%    0.021135082588677;
%    0.002667142882941];

%P =    [0.028129517387884;
%   -0.000000009999994;
%    0.186831727336136;
%    6.740000000000000;
%    0.024106243478703;
%    0.012466142678559];

% P = [pfa,qfa,bfa,VIPF,kcl,rfa];
    %Fast-acting insulin: %Default: Insulin Aspart

%    pfa=0.028604104166292; rfa=0.032657236124532; qfa=-0.000000009999998;
%     bfa=0.368878169545229; kclf =0.024368855208263;
pfa=0.033304427073854; rfa=0.192838157600319; qfa=-0.000000009999983;
    bfa=0.350073112766538; kclf =0.031321989850181;

    %Physical activity parameters: 
    tHR=5; ne=4; ae=0.1; te=600;
    alphae=0.8; betae=3.39e-4; HRb=80;
    ce1 =500; ce2 =100; 
    
    %SMBG sigma:
    sigsmbg = 0.1; 
   end
end