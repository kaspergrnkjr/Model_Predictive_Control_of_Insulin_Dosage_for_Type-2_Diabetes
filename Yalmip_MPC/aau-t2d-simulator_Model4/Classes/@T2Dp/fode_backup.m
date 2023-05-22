%AAU model for simulating a T2D patient.
%This is the definition of the ode function.
function dx=fode(obj,t, x, DgA, stressv, HRv, T, param, basal)
%X is the state vector
%Ula is the long-acting mU
%Ufa is the fast-acting mU
%Um is the Metaformin oral dose mug
%Uv is the Vildagliptin oral dose nmol
%Dg is the amount of oral glucose consumption mg
%stressv is function in time for stress
%HRv is a function in time for HR
%T is a vector of time steps for stress and HR
%Param is a vector for the parameters of the model
%Basal is a vector for the basal values
x = x';
dx = zeros(73,1);
%% Extract the parameters from the parameter vector:

    
    %Glucose absorption model: 
    k12=param.k12; Kq1=param.Kq1; Kq2=param.Kq2; kmin=param.kmin;
    kmax=param.kmax; kabs=param.kabs; fg=param.fg;
    
    k12GIH=param.k12GIH; kabsGIH=param.kabsGIH;

    k12GIM=param.k12GIM; kabsGIM=param.kabsGIM;

    k12GIL=param.k12GIL; kabsGIL=param.kabsGIL;

    k12GIvL=param.k12GIvL; kabsGIvL=param.kabsGIvL;
    %Metformin submodel:
    kgo=param.kgo; kgg=param.kgg; kpg=param.kpg; kgl=param.kgl; kpl=param.kpl;
    klp=param.klp; vGWmax=param.vGWmax; vLmax=param.vLmax; vPmax=param.vPmax;
    nGW=param.nGW; nL=param.nL; nP=param.nP; phiGW50=param.phiGW50;
    phiL50=param.phiL50; phiP50=param.phiP50; rhoO1=param.rhoalpha; rhoO2=param.rhobeta;
    alpham=param.alpham; betam=param.betam; kpo=param.kpo;

    %Vildagliptin submodel: 
    ka1=param.ka1; ka2=param.ka2; CL=param.CL; CLic=param.CLic; Vc=param.Vc;
    Vp=param.Vp; RmaxC=param.RmaxC; kdvil=param.kvd; k2vil=param.k2v;
    koff=param.koff; RmaxP=param.RmaxP; kdeg=param.kdeg;
    
    %Physical activity model:
    tHR=param.tHR; HRb=param.HRb; ne=param.ne; ae=param.ae; te=param.te;
    ce1=param.ce1; ce2=param.ce2;
    %Fast acting insulin:
    pfa=param.pfa; rfa=param.rfa; qfa=param.qfa; bfa=param.bfa;
    kclf = param.kclf;
    
    %Long acting insulin:
    pla=param.pla; rla=param.rla; qla=param.qla; bla=param.bla;...
    Cmax=param.Cmax; kla=param.kla; kcll = param.kcll;

    %Pancreas submodel
    zeta1=param.zeta1; zeta2=param.zeta2; kdmdpan=param.ml0*param.Kl;...
    Kpan=param.Ks; %Kpan
    gammapan=param.gammapan;
    alphapan=param.alphapan;
    betapan=param.betapan; N1=param.N1; N2=param.N2; 
    KILLPAN = param.KILLPAN;
    Sfactor = param.Sfactor;
    %Insulin submodel:
    VIB=param.VIB; VIH=param.VIH; QIB=param.QIB; QIL=param.QIL;
    QIK=param.QIK; QIP=param.QIP; QIH=param.QIH;
    QIG=param.QIG; VIG=param.VIG; VIL=param.VIL; QIA=param.QIA; VIK=param.VIK;
    VIPC=param.VIPC; VIPF=param.VIPF; TIP=param.TIP;


    %Glucose submodel:
    VGBC=param.VGBC; QGB=param.QGB; VGBF=param.VGBF; TGB=param.TGB;
    VGH=param.VGH;  QGL=param.QGL;
    QGK=param.QGK;  QGP=param.QGP; QGH=param.QGH;  VGG=param.VGG;
    QGG=param.QGG;  VGL=param.VGL; QGA=param.QGA;  VGK=param.VGK;
    VGPC=param.VGPC; VGPF=param.VGPF; TGP=param.TGP; alphae=param.alphae;
    betae=param.betae;
    
    %Glucagon submodel:
    VGamma=param.VGamma;

    
    %GLP-1 submodel:
    VPSI=param.VPSI; Kout=param.Kout; CF2=param.CF2;
    tpsi=param.tpsi; kpsi=param.zeta;
    
    %GLP-1 agonists:
    ah2 = param.ah2; bh2 = param.bh2; ch2 = param.ch2; 
    ah24 = param.ah24; bh24 = param.bh24; ch24 = param.ch24;

    %rates:
    cIPGU=param.c1; cIHGPinft=param.c2; 
    cGHGP=param.c3; cIHGUinft=param.c4; cGHGU=param.c5;
    dIPGU=param.d1; dIHGPinft=param.d2; dGHGP=param.d3;
    dIHGUinft=param.d4; dGHGU=param.d5;
    SHGP = param.SHGP; 
    SHGU = param.SHGU; 
    SPGU = param.SPGU; 
    
%% Extract states from state vector:
    %Glucose absorption model: 
    qss=x(1,1); qsl=x(2,1); qint=x(3,1);
    De=x(47,1); DNq=x(48,1);
    qssGIH=x(58,1); qslGIH=x(59,1); qintGIH=x(60,1);
    qssGIM=x(61,1); qslGIM=x(62,1); qintGIM=x(63,1);
    qssGIL=x(64,1); qslGIL=x(65,1); qintGIL=x(66,1);
    qssGIvL=x(67,1); qslGIvL=x(68,1); qintGIvL=x(69,1);

    %Metformin submodel:
    MO1=x(4,1); MO2=x(5,1); MGl=x(6,1); MGW=x(7,1); ML=x(8,1);
    MP=x(9,1);

    %Vildagliptin submodel: 
    AG1=x(10,1); AG2=x(11,1); Ac=x(12,1); Ap=x(13,1); DRc=x(14,1);
    DRp=x(15,1);
    
    %Physical activity model:
    E1=x(16,1); E2=x(17,1);
    TE=x(49,1);
    %Fast acting insulin:
    Hfa=x(18,1); Dfa=x(19,1); Ifa = x(56,1);

    %Long acting insulin:
    Bla=x(20,1); Hla=x(21,1); Dla=x(22,1); Ila = x(55,1);

    %Pancreas submodel
    mpan=x(23,1); P=x(24,1); R=x(25,1);
    
    %Insulin submodel:
    IB=x(26,1);   IH=x(27,1); IG=x(28,1); IL=x(29,1); IK=x(30,1);
    IPC=x(31,1); IPF=x(32,1);

    %Glucose submodel:
    GBC=x(33,1); GBF=x(34,1); GH=x(35,1); GG=x(36,1); GL=x(37,1);...
    GK=x(38,1);  GPC=x(39,1); GPF=x(40,1);
    
    %Glucagon submodel:
    Gamma=x(41,1);
    
    %GLP-1 submodel:    
    psi=x(42,1); PSI=x(43,1);
    
    %GLP-1 agonists

    %Exenatide
    psih2=x(70,1); PSIh2=x(71,1);
    
    %Semaglutide
    psih24=x(72,1); PSIh24=x(73,1);

    %rates:
    MIHGP=x(44,1);
    fr=x(45,1);
    MIHGU=x(46,1);
    
    %Total glucose consumption: 
    XGC = x(50,1);
    %Total Glucose production and appearance: 
    XGP = x(51,1);
    %Total insulin consumption: 
    XIC = x(52,1);
    %Secreted inuslin: 
    XIS = x(53,1);
    %Injected insulin: 
    XIinj = x(54,1);
    
    %Integrated hepatic glucose
    GHint = x(57,1);
%% Basal values and constant rates:
    GBPF=basal.GPF; IBPF=basal.IPF; IBL=basal.IL; GBL=basal.GL;
    GammaB=basal.Gamma; 

    SB=basal.SB;
    GBH=basal.GH;
    IBH=basal.IH;
    rBPIR=basal.rPIR;
    rBGU=basal.rBGU;
    rRBCU=basal.rRBCU;
    rGGU=basal.rGGU;
    rBPGU=basal.rPGU;
    rBHGP=basal.rHGP;
    rBHGU=basal.rHGU;
%% Glucose Absorption submodel
    
    %The model equations
    qssA = qss + qssGIH + qssGIM + qssGIL + qssGIvL;
    qslA = qsl + qslGIH + qslGIM + qslGIL + qslGIvL;
    qintA = qint + qintGIH + qintGIM + qintGIL + qintGIvL; 
    dDe =-kmin*De;
    dDNq = kmin*(DgA-DNq);
    dqss=-k12*qss;
    QA1=5/(2*DNq*(1-Kq1));
    QA2=5/(2*DNq*Kq2);
    kempt=kmin+((kmax-kmin)/2)*(tanh(QA1*(qssA+qslA-Kq1*DNq))...
        -tanh(QA2*(qssA+qslA-Kq2*DNq))+2);
    dqsl=-kempt*qsl+k12*qss;
    dqint=-kabs*qint+kempt*qsl;
    
    %Glucose Absorption (High IG) submodel 
    dqssGIH=-k12GIH*qssGIH;
    dqslGIH=-kempt*qslGIH+k12GIH*qssGIH;
    dqintGIH=-kabsGIH*qintGIH+kempt*qslGIH;
    
    %Glucose Absorption (Medium IG) submodel 
    dqssGIM=-k12GIM*qssGIM;
    dqslGIM=-kempt*qslGIM+k12GIM*qssGIM;
    dqintGIM=-kabsGIM*qintGIM+kempt*qslGIM;
    
    %Glucose Absorption (Low IG) submodel 
    dqssGIL=-k12GIL*qssGIL;
    dqslGIL=-kempt*qslGIL+k12GIL*qssGIL;
    dqintGIL=-kabsGIL*qintGIL+kempt*qslGIL;
    
    %Glucose Absorption (Very low IG) submodel 
    dqssGIvL=-k12GIvL*qssGIvL;
    dqslGIvL=-kempt*qslGIvL+k12GIvL*qssGIvL;
    dqintGIvL=-kabsGIvL*qintGIvL+kempt*qslGIvL;

    
    dx(1,1)=dqss;
    dx(2,1)=dqsl;
    dx(3,1)=dqint;
    dx(47,1)=dDe;
    dx(48,1)=dDNq;
    dx(58,1)=dqssGIH; dx(59,1)=dqslGIH; dx(60,1)=dqintGIH;
    dx(61,1)=dqssGIM; dx(62,1)=dqslGIM; dx(63,1)=dqintGIM;
    dx(64,1)=dqssGIL; dx(65,1)=dqslGIL; dx(66,1)=dqintGIL;
    dx(67,1)=dqssGIvL; dx(68,1)=dqslGIvL; dx(69,1)=dqintGIvL;
    Ra=fg*kabs*qint + fg*kabsGIH*qintGIH + fg*kabsGIM*qintGIM + fg*kabsGIL*qintGIL + fg*kabsGIvL*qintGIvL;

%% Stress:
%Stress is a parameter that takes values between 1 and 0.
%It is defined as a vector of a sepcific time step for the simulation.
%Therefore, it is necessary to interpolate it here:
stress=0; %stressv(t);%interp1(0:obj.dt:(length(stressv)*obj.dt-obj.dt),stressv,t); %Interpolates (T,stressv) at time t

%% Metformin submodel: 

    %Model equations:
    dMO1=-alpham*MO1;
    dMO2=-betam*MO2;
    dMGl=-(kgo+kgg)*MGl+rhoO1*MO1+rhoO2*MO2;
    dMGW=MGl*kgg+MP*kpg-MGW*kgl;
    dML=MGW*kgl+MP*kpl-ML*klp;
    dMP=ML*klp-(kpl+kpg+kpo)*MP+MGl;

    dx(4,1)=dMO1; dx(5,1)=dMO2; dx(6,1)=dMGl; dx(7,1)=dMGW; 
    dx(8,1)=dML; dx(9,1)=dMP;
    EGW=(vGWmax*(MGW)^(nGW))/(phiGW50^(nGW)+(MGW)^(nGW));
    EL=(vLmax*(ML)^(nL))/(phiL50^(nL)+(ML)^(nL));
    EP=(vPmax*(MP)^(nP))/(phiP50^(nP)+(MP)^(nP));
%% Vildagliptin submodel:

    %The model equations:
    dAG1=-ka1*AG1;
    dAG2=ka1*AG1-ka2*AG2;
    dAc=ka2*AG2-((CL+CLic)/Vc)*Ac+(CLic/Vp)*Ap-((RmaxC-DRc)*k2vil*(Ac/Vc))/(kdvil+Ac/Vc)+koff*DRc;
    dAP=CLic*(Ac/Vc-Ap/Vp)-((RmaxP-DRp)*k2vil*(Ap/Vp))/(kdvil+Ap/Vp)+koff*DRp;
    dDRc=(RmaxC-DRc)*k2vil*(Ac/Vc)/(kdvil+Ac/Vc)-(koff-kdeg)*DRc;
    dDRp=(RmaxP-DRp)*k2vil*(Ap/Vp)/(kdvil+Ap/Vp)-(koff+kdeg)*DRp;
    dx(10,1)=dAG1; dx(11,1)=dAG2; dx(12,1)=dAc; dx(13,1)=dAP;
    dx(14,1)=dDRc; dx(15,1)=dDRp;


 %% GLP-1 agonists 
    
    %(Exenatide):
    dpsih2 = -ah2*psih2;
    dPSIh2 = bh2*ah2*psih2 - bh2*PSIh2;
    dx(70,1) = dpsih2;
    dx(71,1) = dPSIh2;
    
    %(Semaglutide):
    dpsih24 = -ah24*psih24;
    dPSIh24 = bh24*ah24*psih24 - bh24*PSIh24;
    dx(72,1) = dpsih24;
    dx(73,1) = dPSIh24;
%% Physical activity submodel
    HR=HRb;%HRv(t);%interp1(0:obj.dt:(length(HRv)*obj.dt-obj.dt),HRv,t); %Interpolates (T,HRv) at time t
    %The model equations:
    dE1=(1/tHR)*(HR-HRb-E1);
    gE=(E1/(ae*HRb))^ne/(1+(E1/(ae*HRb))^ne);
    dTE = (1/te)*(ce1*gE+ce2-TE);
    %dE2 = -(gE+1/TE)*E2+(gE*TE)/(ce1+ce2);
    dE2=-(gE+1/te)*E2+gE;
    dx(16,1)=dE1;
    dx(17,1)=dE2;
    dx(49,1)=dTE;
%% Glucose submodel rates: 
    
    MIPGU=(7.03+SPGU*6.52*tanh(cIPGU*(IPF/IBPF-dIPGU)))/(7.03+SPGU*6.52*tanh(cIPGU*(1-dIPGU)));
    
    MGPGU=GPF/GBPF;
    
    rPGU=MIPGU*MGPGU*rBPGU;
    
    
    MIHGPinft=(1.21-SHGP*1.14*tanh(cIHGPinft*(IL/IBL-dIHGPinft)))/...
        (1.21-SHGP*1.14*tanh(cIHGPinft*(1-dIHGPinft)));
    
    MGHGP=(1.42-1.41*tanh(cGHGP*(GL/GBL-dGHGP)))...
        /(1.42-1.41*tanh(cGHGP*(1-dGHGP)));
    
    MgammaHGP=2.7*tanh(0.39*Gamma/GammaB)-fr;
    
    rHGP=MIHGP*MGHGP*MgammaHGP*rBHGP;
        
    MIHGUinft=(tanh(cIHGUinft*(IL/IBL-dIHGUinft)))...
    /(tanh(cIHGUinft*(1-dIHGUinft)));

    MGHGU=(5.66+5.66*tanh(cGHGU*(GL/GBL-dGHGU)))...
        /(5.66+5.66*tanh(cGHGU*(1-dGHGU)));
    
    rHGU=MIHGU*MGHGU*rBHGU;

%     if GK>=460
%         rKGE=330+0.872*GK;
%     else
%         rKGE=71+71*tanh(0.011*(GK-460));
%     end
   rKGE=(330+0.872*GK).*(1./(1+exp(-0.05*(GK-460)))) + (71+71*tanh(0.011*(GK-460))).*(1./(1+exp(0.05*(GK-460))));
    %Effect of Metformin:
    rHGP=rHGP*(1-EL);
    rGGU=rGGU*(1+EGW);
    rPGU=rPGU*(1+EP);
%%  Rates dynamic model
    
    dMIHGP=0.04*(MIHGPinft-MIHGP);
    dfr=0.0154*(0.5*(2.7*tanh(0.39*Gamma/GammaB)-1)-fr);
    dMIHGU=0.04*(MIHGUinft-MIHGU);
    
    dx(44,1)=dMIHGP;
    dx(45,1)=dfr;
    dx(46,1)=dMIHGU;
%%  Glucose submodel:
    %+564.4444
    dGBC=(1/VGBC)*(QGB*(GH-GBC)-(VGBF/TGB)*(GBC-GBF));
    dGBF=(1/VGBF)*((VGBF/TGB)*(GBC-GBF)-rBGU);
    dGH=(1/VGH)*(QGB*GBC+QGL*GL+QGK*GK+QGP*GPC-QGH*GH-rRBCU);
    dGG=(1/VGG)*(QGG*(GH-GG)-rGGU+Ra);
    dGL=(1/VGL)*(QGA*GH+QGG*GG-QGL*GL+((1+stress)*(1-alphae*E2)*rHGP-(1+alphae*E2)*rHGU));
    dGK=(1/VGK)*(QGK*(GH-GK)-rKGE);
    dGPC=(1/VGPC)*(QGP*(GH-GPC)-(VGPF/TGP)*(GPC-GPF));
    dGPF=(1/VGPF)*((VGPF/TGP)*(GPC-(1+betae*E1)*GPF)-(1+alphae*E2)*rPGU);
    
    dx(33,1)=dGBC; dx(34,1)=dGBF; dx(35,1)=dGH; dx(36,1)=dGG;
    dx(37,1)=dGL; dx(38,1)=dGK; dx(39,1)=dGPC; dx(40,1)=dGPF;
    dXGC = (rBGU)+rRBCU+rGGU+(1+alphae*E2)*rHGU+rKGE+betae*E1*GPF*QGP+...
        (1+alphae*E2)*rPGU;
    dXGP = Ra + (1+stress)*(1-alphae*E2)*rHGP;
    dGHint = GH;
    dx(50,1) = dXGC;
    dx(51,1) = dXGP;
    dx(57,1) = dGHint;
%% Glucagon submodel:
   
   %Basal values: 
   rBPGammaR=9.1;
   
   MGPGammaR=1.31-0.61*tanh(1.06*((GH/GBH)-0.47));
   MIPGammaR=2.93-2.09*tanh(4.18*((IH/IBH)-0.62));
   
   rPGammaR=MGPGammaR*MIPGammaR*rBPGammaR;
   
   dGamma=(1/VGamma)*((1+stress)*rPGammaR-9.1*Gamma);
   dx(41,1)=dGamma;

%% GLP-1 submodel

    dpsi=kpsi*kempt*qslA-psi/tpsi;
    dPSI=(1/VPSI)*(psi/tpsi-(Kout+(RmaxC-DRc)*CF2)*PSI);

    dx(42,1)=dpsi;
    dx(43,1)=dPSI;

%% Pancreas submodel

    %The model equations:
    XG=GH^(3.27)/(1.32^3.27+5.93*GH^3.02);
    Pinft=XG^(1.11)+zeta1*(PSI+ch24*PSIh24+ch2*PSIh2);
    Y=Pinft;
%     S=mpan*(N1*Y+N2*(XG-R)+zeta2*PSI);
%     if XG>R
%         S=Sfactor*mpan*(N1*Y+N2*(XG-R)+zeta2*PSI);
%     else
%         S=Sfactor*mpan*(N1*Y+zeta2*PSI);
%     end
    S=Sfactor*mpan*((N1*Y+zeta2*(PSI+ch24*PSIh24+ch2*PSIh2)).*(1./(1+exp(0.5*(XG-R))))+...
      (N1*Y+N2*(XG-R)+zeta2*(PSI+ch24*PSIh24+ch2*PSIh2)).*(1./(1+exp(-0.5*(XG-R)))));
    dmpan=kdmdpan-Kpan*mpan+gammapan*P-S;
    dP=alphapan*(Pinft-P);
    dR=betapan*(XG-R);
    dx(23,1)=dmpan; dx(24,1)=dP; dx(25,1)=dR;
    
%% Insulin submodel rates:
    rPIR=(S/SB)*rBPIR;
    rLIC=0.4*(QIA*IH+QIG*IG+rPIR);
    rKIC=0.3*QIK*IK;
    rPIC=IPF/((0.85)/(0.15*QIP)-20/VIPF);
%% Long-acting Insulin:

    %Model equations:
    dBla=-kla*Bla*(Cmax/(1+Hla));
    dHla=-pla*(Hla-qla*Dla^3)+kla*Bla*(Cmax/(1+Hla));
    dDla=pla*(Hla-qla*Dla^3)-bla*Dla/(1+Ila);
    dIla =  rla*bla*Dla/(1+(Ila)) - kcll*(Ila);
    dx(20,1)=dBla; dx(21,1)=dHla; dx(22,1)=dDla;
    dx(55,1) = dIla;
%% Fast-acting Insulin 

    %Model equations:
    dHfa=-pfa*(Hfa-qfa*Dfa^3);
    dDfa=pfa*(Hfa-qfa*Dfa^3)-bfa*Dfa/(1+Ifa);
    dIfa =  rfa*bfa*Dfa/(1+(Ifa)) - kclf*(Ifa);
    dx(18,1)=dHfa; dx(19,1)=dDfa; dx(56,1) = dIfa;
%% Insulin submodel: 

    %Model equations:
    dIB=(QIB/VIB)*(IH-IB);
    dIH=(1/VIH)*(QIB*IB+QIL*IL+QIK*IK+QIP*IPF-QIH*IH);
    dIG=(QIG/VIG)*(IH-IG);
    dIL=(1/VIL)*(QIA*IH+QIG*IG-QIL*IL+(1-stress)*rPIR-rLIC);
    dIK=(1/VIK)*(QIK*(IH-IK)-rKIC);
    dIPC=(1/VIPC)*(QIP*(IH-IPC)-(VIPF/TIP)*(IPC-IPF))+10*Ifa+10*Ila;
    dIPF=(1/VIPF)*((VIPF/TIP)*(IPC-IPF)-rPIC);
    dx(26,1)=dIB; dx(27,1)=dIH; dx(28,1)=dIG; dx(29,1)=dIL; dx(30,1)=dIK;
    dx(31,1)=dIPC; dx(32,1)=dIPF;
    dXIC = rLIC+rKIC+rPIC;
    dXIS = (1-stress)*rPIR;
    dXIinj = VIPF*rla*bla*Dla/(1+IPF)+VIPF*rfa*bfa*Dfa/(1+IPF);
    dx(52,1) = dXIC; 
    dx(53,1) = dXIS;
    dx(54,1) = dXIinj;
end