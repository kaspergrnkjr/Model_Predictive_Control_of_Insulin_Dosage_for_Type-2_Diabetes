%AAU model for simulating a T2D patient.
%This is the definition of the ode function.

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

%% Extract the parameters from the parameter vector:
clear all 
close all 
    
    %Glucose absorption model: 
    syms k12 Kq1 Kq2 kmin kmax kabs fg k12GIH kabsGIH k12GIM...
        kabsGIM k12GIL kabsGIL k12GIvL kabsGIvL real;
  
    %Metformin submodel:
    syms kgo kgg kpg kgl kpl klp vGWmax vLmax vPmax nGW nL ...
        nP phiGW50 phiL50 phiP50 rhoO1 rhoO2 alpham betam kpo real;

    %Vildagliptin submodel: 
    syms ka1 ka2 CL CLic Vc Vp RmaxC kdvil k2vil koff RmaxP kdeg real;
    
    %Physical activity model:
    syms tHR HRb ne ae te ce1 ce2 real;

    %Fast acting insulin:
    syms pfa rfa qfa bfa kclf rea;
    
    %Long acting insulin:
    syms pla rla qla bla...
    Cmax kla kcll real 

    %Pancreas submodel
    syms zeta1 zeta2 kdmdpan...
    Kpan gammapan alphapan betapan N1 N2 KILLPAN Sfactor real;
    
    %Insulin submodel:
    syms VIB VIH QIB QIL ...
    QIK QIP QIH ... 
    QIG VIG VIL QIA VIK ...
    VIPC VIPF TIP real;


    %Glucose submodel:
    syms VGBC QGB VGBF TGB ...
    VGH  QGL ...
    QGK  QGP QGH  VGG ...
    QGG  VGL QGA  VGK ...
    VGPC VGPF TGP alphae ....
    betae real;
    
    %Glucagon submodel:
    syms VGamma real

    
    %GLP-1 submodel:
    syms VPSI Kout CF2 ...
    tpsi kpsi real; 
    
    %GLP-1 agonists:
    syms ah2 bh2 ch2  ... 
    ah24 bh24 ch24  real;

    %rates:
    syms cIPGU cIHGPinft ...
    cGHGP cIHGUinft cGHGU ...
    dIPGU dIHGPinft dGHGP ...
    dIHGUinft dGHGU ...
    SHGP SHGU SPGU real; 
    
%% Extract states from state vector:
    %Glucose absorption model: 
    syms qss qsl qint De DNq ...
    qssGIH qslGIH qintGIH ...
    qssGIM qslGIM qintGIM ...
    qssGIL qslGIL qintGIL ...
    qssGIvL qslGIvL qintGIvL real; 

    x(1,1)=qss; x(2,1)=qsl; x(3,1)=qint;
    x(47,1)=De; x(48,1)=DNq;
    x(58,1)=qssGIH; x(59,1)=qslGIH; x(60,1)=qintGIH;
    x(61,1)=qssGIM; x(62,1)=qslGIM; x(63,1)=qintGIM;
    x(64,1)=qssGIL; x(65,1)=qslGIL; x(66,1)=qintGIL;
    x(67,1)=qssGIvL; x(68,1)=qslGIvL; x(69,1)=qintGIvL;

    %Metformin submodel:
    syms MO1 MO2 MGl MGW ML MP real;
    x(4,1)=MO1; x(5,1)=MO2; x(6,1)=MGl; x(7,1)=MGW; x(8,1)=ML;
    x(9,1)=MP;

    %Vildagliptin submodel: 
    syms AG1 AG2 Ac Ap DRc DRp real;
    x(10,1)=AG1; x(11,1)=AG2; x(12,1)=Ac; x(13,1)=Ap; x(14,1)=DRc;
    x(15,1)=DRp;
    
    %Physical activity model:
    syms E1 E2 TE real;
    x(16,1)=E1; x(17,1)=E2;
    x(49,1)=TE;

    %Fast acting insulin:
    syms Hfa Dfa Ifa real ;
    x(18,1)=Hfa; x(19,1)=Dfa; x(56,1) = Ifa;

    %Long acting insulin:
    syms Bla Hla Dla Ila real;
    x(20,1)=Bla; x(21,1)=Hla; x(22,1)=Dla;  x(55,1)=Ila;

    %Pancreas submodel
    syms mpan P R real;
    x(23,1)=mpan; x(24,1)=P; x(25,1)=R;
    
    %Insulin submodel:
    syms IB IH IG IL IK IPC IPF real;

    x(26,1)=IB;   x(27,1) = IH; x(28,1)=IG; x(29,1)=IL; x(30,1)=IK;
    x(31,1)=IPC; x(32,1)=IPF;

    %Glucose submodel:
    syms GBC GBF GH GG GL GK GPC GPF real;
    x(33,1)=GBC; x(34,1)=GBF; x(35,1)=GH; x(36,1)=GG; x(37,1)=GL;...
    x(38,1)=GK;  x(39,1)=GPC; x(40,1)=GPF;
    
    %Glucagon submodel:
    syms Gamma real;
    x(41,1)=Gamma;
    
    %GLP-1 submodel:
    syms psi PSI real;
    x(42,1)=psi; x(43,1)=PSI;
    
    %GLP-1 agonists

    %Exenatide
    syms psih2 PSIh2 real 
    x(70,1)=psih2; x(71,1)=PSIh2;
    
    %Semaglutide
    syms psih24 PSIh24 real 
    x(72,1)=psih24; x(73,1)=PSIh24;

    %rates:
    syms MIHGP fr MIHGU real 

    x(44,1)=MIHGP;
    x(45,1)=fr;
    x(46,1)=MIHGU;
    
    %Total glucose consumption: 
    syms XGC XGP XIC XIS XIinj GHint real 
    x(50,1)=XGC;
    %Total Glucose production and appearance: 
    x(51,1)=XGP;
    %Total insulin consumption: 
    x(52,1)=XIC;
    %Secreted inuslin: 
    x(53,1)=XIS;
    %Injected insulin: 
    x(54,1)=XIinj;
    
    %Integrated hepatic glucose
    x(57,1)=GHint;

    syms DgA stressv HRv real 
%% Basal values and constant rates:
     
    syms GBPF IBPF IBL GBL GammaB  ...
    SB GBH IBH rBPIR rBGU rRBCU rGGU rBPGU rBHGP rBHGU real 
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
stress=stressv;%interp1(0:obj.dt:(length(stressv)*obj.dt-obj.dt),stressv,t); %Interpolates (T,stressv) at time t

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
    HR=HRv;%interp1(0:obj.dt:(length(HRv)*obj.dt-obj.dt),HRv,t); %Interpolates (T,HRv) at time t
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
   rKGE=(330+0.872*GK).*(1./(1+exp(-0.02*(GK-460)))) + (71+71*tanh(0.011*(GK-460))).*(1./(1+exp(0.02*(GK-460))));
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
    S=Sfactor*mpan*((N1*Y+zeta2*(PSI+ch24*PSIh24+ch2*PSIh2)).*(1./(1+exp(0.1*(XG-R))))+...
      (N1*Y+N2*(XG-R)+zeta2*(PSI+ch24*PSIh24+ch2*PSIh2)).*(1./(1+exp(-0.1*(XG-R)))));
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
    

    %Jacobian: 
    J = jacobian(dx,x);
    Jfode = matlabFunction(J,'File','jacobian_fode','sparse',true);

