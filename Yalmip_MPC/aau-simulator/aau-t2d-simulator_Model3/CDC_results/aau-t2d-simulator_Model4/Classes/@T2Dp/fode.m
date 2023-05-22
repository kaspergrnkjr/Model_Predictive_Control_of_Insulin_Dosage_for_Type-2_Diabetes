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
    k12=param(39); Kq1=param(37); Kq2=param(38); kmin=param(40);
    kmax=param(41); kabs=param(42); fg=param(36);
    
    k12GIH=param(43); kabsGIH=param(44);

    k12GIM=param(45); kabsGIM=param(46);

    k12GIL=param(47); kabsGIL=param(48);

    k12GIvL=param(49); kabsGIvL=param(50);

    %Metformin submodel:
    
    kgo=param(100); kgg=param(101); kpg=param(102); kgl=param(103); kpl=param(104);
    klp=param(105); vGWmax=param(107); vLmax=param(108); vPmax=param(109);
    nGW=param(110); nL=param(111); nP=param(112); phiGW50=param(113);
    phiL50=param(114); phiP50=param(115); rhoO1=param(116); rhoO2=param(117);
    alpham=param(118); betam=param(119); kpo=param(106);

    %Vildagliptin submodel: 
    ka1=param(89); ka2=param(90); CL=param(91); CLic=param(92); Vc=param(94);
    Vp=param(93); RmaxC=param(81); kdvil=param(95); k2vil=param(96);
    koff=param(97); RmaxP=param(98); kdeg=param(99);
    
    %Physical activity model:
    tHR=param(132); HRb=param(138); ne=param(133); ae=param(134); te=param(135);
    ce1=param(139); ce2=param(140);
    
    %Fast acting insulin:
    pfa=param(127); rfa=param(128); qfa=param(129); bfa=param(130);
    kclf = param(131);
    
    %Long acting insulin:
    pla=param(120); rla=param(121); qla=param(122); bla=param(123);...
    Cmax=param(124); kla=param(125); kcll = param(126);

    %Pancreas submodel
    zeta1=param(64); zeta2=param(65); kdmdpan=param(66)*param(67);...
    Kpan=param(68); %Kpan
    gammapan=param(69);
    alphapan=param(70);
    betapan=param(71); N1=param(72); N2=param(73); 
    KILLPAN = param(74);
    Sfactor = param(75);

    %Insulin submodel:
    VIB=param(19); VIH=param(20); QIB=param(25); QIL=param(33);
    QIK=param(28); QIP=param(29); QIH=param(26);
    QIG=param(30); VIG=param(21); VIL=param(22); QIA=param(27); VIK=param(23);
    VIPC=param(34); VIPF=param(24); TIP=param(31);


    %Glucose submodel:
    VGBC=param(1); QGB=param(9); VGBF=param(2); TGB=param(16);
    VGH=param(3);  QGL=param(12);
    QGK=param(14);  QGP=param(15); QGH=param(10);  VGG=param(5);
    QGG=param(13);  VGL=param(4); QGA=param(11);  VGK=param(6);
    VGPC=param(7); VGPF=param(8); TGP=param(17); alphae=param(136);
    betae=param(137);
    
    %Glucagon submodel:
    VGamma=param(35);

    
    %GLP-1 submodel:
    VPSI=param(76); Kout=param(77); CF2=param(78);
    tpsi=param(79); kpsi=param(80);
    
    %GLP-1 agonists:
    ah2 = param(82); bh2 = param(83); ch2 = param(84); 
    ah24 = param(85); bh24 = param(86); ch24 = param(87);

    %rates:
    cIPGU=param(51); cIHGPinft=param(52); 
    cGHGP=param(53); cIHGUinft=param(54); cGHGU=param(55);
    dIPGU=param(56); dIHGPinft=param(57); dGHGP=param(58);
    dIHGUinft=param(59); dGHGU=param(60);
    SHGP = param(62); 
    SHGU = param(61); 
    SPGU = param(63); 
    
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
    GBPF=basal(1); IBPF=basal(2); IBL=basal(3); GBL=basal(4);
    GammaB=basal(5); 

    SB=basal(6);
    GBH=basal(7);
    IBH=basal(8);
    rBPIR=basal(9);
    rBGU=basal(10);
    rRBCU=basal(11);
    rGGU=basal(12);
    rBPGU=basal(13);
    rBHGP=basal(14);
    rBHGU=basal(15);
%% Glucose Absorption submodel
    
    %The model equations
    qssA = qss + qssGIH + qssGIM + qssGIL + qssGIvL;
    qslA = qsl + qslGIH + qslGIM + qslGIL + qslGIvL;
    qintA = qint + qintGIH + qintGIM + qintGIL + qintGIvL; 
    dx(47) =-kmin*De;
    dx(48) = kmin*(DgA-DNq);
    dx(1)=-k12*qss;
    QA1=5/(2*DNq*(1-Kq1));
    QA2=5/(2*DNq*Kq2);
    kempt=kmin+((kmax-kmin)/2)*(tanh(QA1*(qssA+qslA-Kq1*DNq))...
        -tanh(QA2*(qssA+qslA-Kq2*DNq))+2);
    dx(2)=-kempt*qsl+k12*qss;
    dx(3)=-kabs*qint+kempt*qsl;
    
    %Glucose Absorption (High IG) submodel 
    dx(58)=-k12GIH*qssGIH;
    dx(59)=-kempt*qslGIH+k12GIH*qssGIH;
    dx(60)=-kabsGIH*qintGIH+kempt*qslGIH;
    
    %Glucose Absorption (Medium IG) submodel 
    dx(61)=-k12GIM*qssGIM;
    dx(62)=-kempt*qslGIM+k12GIM*qssGIM;
    dx(63)=-kabsGIM*qintGIM+kempt*qslGIM;
    
    %Glucose Absorption (Low IG) submodel 
    dx(64)=-k12GIL*qssGIL;
    dx(65)=-kempt*qslGIL+k12GIL*qssGIL;
    dx(66)=-kabsGIL*qintGIL+kempt*qslGIL;
    
    %Glucose Absorption (Very low IG) submodel 
    dx(67)=-k12GIvL*qssGIvL;
    dx(68)=-kempt*qslGIvL+k12GIvL*qssGIvL;
    dx(69)=-kabsGIvL*qintGIvL+kempt*qslGIvL;

    
    Ra=(fg*kabs*qint + fg*kabsGIH*qintGIH + fg*kabsGIM*qintGIM + fg*kabsGIL*qintGIL + fg*kabsGIvL*qintGIvL);

%% Stress:
%Stress is a parameter that takes values between 1 and 0.
%It is defined as a vector of a sepcific time step for the simulation.
%Therefore, it is necessary to interpolate it here:
stress=0; %stressv(t);%interp1(0:obj.dt:(length(stressv)*obj.dt-obj.dt),stressv,t); %Interpolates (T,stressv) at time t

%% Metformin submodel: 

    %Model equations:
    dx(4)=-alpham*MO1;
    dx(5)=-betam*MO2;
    dx(6)=-(kgo+kgg)*MGl+rhoO1*MO1+rhoO2*MO2;
    dx(7)=MGl*kgg+MP*kpg-MGW*kgl;
    dx(8)=MGW*kgl+MP*kpl-ML*klp;
    dx(9)=ML*klp-(kpl+kpg+kpo)*MP+MGl;

    
    EGW=(vGWmax*(MGW)^(nGW))/(phiGW50^(nGW)+(MGW)^(nGW));
    EL=(vLmax*(ML)^(nL))/(phiL50^(nL)+(ML)^(nL));
    EP=(vPmax*(MP)^(nP))/(phiP50^(nP)+(MP)^(nP));
%% Vildagliptin submodel:

    %The model equations:
    dx(10)=-ka1*AG1;
    dx(11)=ka1*AG1-ka2*AG2;
    dx(12)=ka2*AG2-((CL+CLic)/Vc)*Ac+(CLic/Vp)*Ap-((RmaxC-DRc)*k2vil*(Ac/Vc))/(kdvil+Ac/Vc)+koff*DRc;
    dx(13)=CLic*(Ac/Vc-Ap/Vp)-((RmaxP-DRp)*k2vil*(Ap/Vp))/(kdvil+Ap/Vp)+koff*DRp;
    dx(14)=(RmaxC-DRc)*k2vil*(Ac/Vc)/(kdvil+Ac/Vc)-(koff-kdeg)*DRc;
    dx(15)=(RmaxP-DRp)*k2vil*(Ap/Vp)/(kdvil+Ap/Vp)-(koff+kdeg)*DRp;



 %% GLP-1 agonists 
    
    dx(70) = 0;
    dx(71) = 0;

    
    dx(72) = 0;
    dx(73) = 0;

%% Physical activity submodel
    HR=HRb;%HRv(t);%interp1(0:obj.dt:(length(HRv)*obj.dt-obj.dt),HRv,t); %Interpolates (T,HRv) at time t
    %The model equations:
    dx(16)=(1/tHR)*(HR-HRb-E1);
    gE=(E1/(ae*HRb))^ne/(1+(E1/(ae*HRb))^ne);
    dx(49) = 0;%(1/te)*(ce1*gE+ce2-TE);
    %dx(17) = -(gE+1/TE)*E2+(gE*TE)/(ce1+ce2);
    dx(17)=-(gE+1/te)*E2+gE;

%% Glucose submodel rates: 
    
    MIPGU=SPGU*(7.03+6.52*tanh(cIPGU*(IPF/IBPF-dIPGU)))/(7.03+6.52*tanh(cIPGU*(1-dIPGU)));
    
    MGPGU=GPF/GBPF;
    
    rPGU=MIPGU*MGPGU*rBPGU;
    
    
    MIHGPinft=(1.21-1.14*tanh(cIHGPinft*(IL/IBL-dIHGPinft)))/...
        (1.21-1.14*tanh(cIHGPinft*(1-dIHGPinft)))/(1+SHGP/5*log(1+exp(5*(IL/IBL-1))));
    
    MGHGP=(1.42-1.41*tanh(cGHGP*(GL/GBL-dGHGP)))...
        /(1.42-1.41*tanh(cGHGP*(1-dGHGP)));
    
    MgammaHGP=2.7*tanh(0.39*Gamma/GammaB)-fr;
    
    rHGP=MIHGP*MGHGP*MgammaHGP*rBHGP;
        
    MIHGUinft=(tanh(cIHGUinft*(IL/IBL-dIHGUinft)))...
    /(tanh(cIHGUinft*(1-dIHGUinft)))+(SHGU*1/5)*(log(1+exp(5*(IL/IBL-1))));

    MGHGU=(5.66+5.66*tanh(cGHGU*(GL/GBL-dGHGU)))...
        /(5.66+5.66*tanh(cGHGU*(1-dGHGU)));
    
    rHGU=MIHGU*MGHGU*rBHGU;

%     if GK>=460
%         rKGE=330+0.872*GK;
%     else
%         rKGE=71+71*tanh(0.011*(GK-460));
%     end
   rKGE=(330+0.872*GK).*(1./(1+exp(-0.02*(GK-460)))) + (71+71*tanh(0.011*(GK-460))).*(1./(1+exp(0.02*(GK-460))));
    %Effect of Metformin:
    rHGP=rHGP*(1-EL);
    rGGU=rGGU*(1+EGW);
    rPGU=rPGU*(1+EP);
%%  Rates dynamic model
    
    dx(44)=0.001*(MIHGPinft-MIHGP);
    dx(45)=0.0154*(0.5*(2.7*tanh(0.39*Gamma/GammaB)-1)-fr);
    dx(46)=0.04*(MIHGUinft-MIHGU);
    
%%  Glucose submodel:
    %+564.4444
    dx(33)=(1/VGBC)*(QGB*(GH-GBC)-(VGBF/TGB)*(GBC-GBF));
    dx(34)=(1/VGBF)*((VGBF/TGB)*(GBC-GBF)-rBGU);
    dx(35)=(1/VGH)*(QGB*GBC+QGL*GL+QGK*GK+QGP*GPC-QGH*GH-rRBCU);
    dx(36)=(1/VGG)*(QGG*(GH-GG)-rGGU+Ra);
    dx(37)=(1/VGL)*(QGA*GH+QGG*GG-QGL*GL+((1+stress)*(1-alphae*E2)*rHGP-(1+alphae*E2)*rHGU));
    dx(38)=(1/VGK)*(QGK*(GH-GK)-rKGE);
    dx(39)=(1/VGPC)*(QGP*(GH-GPC)-(VGPF/TGP)*(GPC-GPF));
    dx(40)=(1/VGPF)*((VGPF/TGP)*(GPC-(1+betae*E1)*GPF)-(1+alphae*E2)*rPGU);
    

    dx(50) = (rBGU)+rRBCU+rGGU+(1+alphae*E2)*rHGU+rKGE+betae*E1*GPF*QGP+...
        (1+alphae*E2)*rPGU;
    dx(51) = Ra + (1+stress)*(1-alphae*E2)*rHGP;
    dx(57) = GH;

%% Glucagon submodel:
   
   %Basal values: 
   rBPGammaR=9.1;
   
   MGPGammaR=(1.31-0.61*tanh(1.06*((GH/GBH)-0.47)));%/0.9993;
   MIPGammaR=(2.93-2.09*tanh(4.18*((IH/IBH)-0.62)));%/1.0074;
   
   rPGammaR=MGPGammaR*MIPGammaR*rBPGammaR;
   
   dx(41)=(1/VGamma)*((1+stress)*rPGammaR-9.1*Gamma);

%% GLP-1 submodel

    dx(42)=kpsi*kempt*qslA-psi/tpsi;
    dx(43)=(1/VPSI)*(psi/tpsi-(Kout+(RmaxC-DRc)*CF2)*PSI);


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
    S=Sfactor*mpan*((N1*Y+zeta2*(PSI+ch24*PSIh24+ch2*PSIh2)).*(1./(1+exp(0.1*(XG-R))))+...
      (N1*Y+N2*(XG-R)+zeta2*(PSI+ch24*PSIh24+ch2*PSIh2)).*(1./(1+exp(-0.1*(XG-R)))));
    dx(23)=kdmdpan-Kpan*mpan+gammapan*P-S;
    dx(24)=alphapan*(Pinft-P);
    dx(25)=betapan*(XG-R);
    
%% Insulin submodel rates:
    rPIR=(S/SB)*rBPIR;
    rLIC=0.4*(QIA*IH+QIG*IG+rPIR);
    rKIC=0.3*QIK*IK;
    rPIC=IPF/((0.85)/(0.15*QIP)-20/VIPF);
%% Long-acting Insulin:

    %Model equations:
    dx(20)=-kla*Bla*(Cmax/(1+Hla));
    dx(21)=-pla*(Hla-qla*Dla^3)+kla*Bla*(Cmax/(1+Hla));
    dx(22)=pla*(Hla-qla*Dla^3)-bla*Dla/(1+Ila);
    dx(55) =  rla*bla*Dla/(1+(Ila)) - kcll*(Ila);

%% Fast-acting Insulin 

    %Model equations:
    dx(18)=-pfa*(Hfa-qfa*Dfa^3);
    dx(19)=pfa*(Hfa-qfa*Dfa^3)-bfa*Dfa/(1+Ifa);
    dx(56) =  rfa*bfa*Dfa/(1+(Ifa)) - kclf*(Ifa);
%% Insulin submodel: 

    %Model equations:
    dx(26)=(QIB/VIB)*(IH-IB);
    dx(27)=(1/VIH)*(QIB*IB+QIL*IL+QIK*IK+QIP*IPF-QIH*IH);
    dx(28)=(QIG/VIG)*(IH-IG);
    dx(29)=(1/VIL)*(QIA*IH+QIG*IG-QIL*IL+(1-stress)*rPIR-rLIC);
    dx(30)=(1/VIK)*(QIK*(IH-IK)-rKIC);
    dx(31)=(1/VIPC)*(QIP*(IH-IPC)-(VIPF/TIP)*(IPC-IPF))+10*Ifa+10*Ila;
    dx(32)=(1/VIPF)*((VIPF/TIP)*(IPC-IPF)-rPIC);

    dx(52) = rLIC+rKIC+rPIC;
    dx(53) = (1-stress)*rPIR;
    dx(54) = VIPF*rla*bla*Dla/(1+IPF)+VIPF*rfa*bfa*Dfa/(1+IPF);
end