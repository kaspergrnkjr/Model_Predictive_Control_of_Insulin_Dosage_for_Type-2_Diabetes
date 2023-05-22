%This function calculate the initial values\ basal values given GPF and IPF
function [x,rates,S]=BasaldefGCPFIPF(obj)
param=obj.Param;
GPC = obj.GBPC0; 
IPF = obj.IBPF0;
rBGU = obj.brates.rBGU;
rRBCU = obj.brates.rRBCU;
rGGU = obj.brates.rGGU;
rPGU = obj.brates.rPGU;
rHGU = obj.brates.rHGU;
%% Extract parameters:
    %Glucose absorption model: 
    k12=param.k12; Kq1=param.Kq1; Kq2=param.Kq2; kmin=param.kmin;
    kmax=param.kmax; kabs=param.kabs; fg=param.fg;

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
    
    %Long acting insulin:
    pla=param.pla; rla=param.rla; qla=param.qla; bla=param.bla;...
    Cmax=param.Cmax; kla=param.kla;

    %Pancreas submodel
    zeta1=param.zeta1; zeta2=param.zeta2; kdmdpan=param.ml0*param.Kl;...
    Kpan=param.Ks; %Kpan
    gammapan=param.gammapan; alphapan=param.alphapan;
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
    
    %rates:
    cIPGU=param.c1; cIHGPinft=param.c2; 
    cGHGP=param.c3; cIHGUinft=param.c4; cGHGU=param.c5;
    dIPGU=param.d1; dIHGPinft=param.d2; dGHGP=param.d3;
    dIHGUinft=param.d4; dGHGU=param.d5;
%% GLucose basal values:
    %rHGP=35;
    GPF=GPC-TGP*rPGU/VGPF;
    GH=GPC+(VGPF/(QGP*TGP))*(GPC-GPF);
    GG=GH-rGGU/QGG;
    GBC=GH-(1/QGB)*rBGU;
    GBF=GBC-(TGB/VGBF)*rBGU;
    rKGE = @(GK) (330+0.872*GK).*(1./(1+exp(-0.02*(GK-460)))) + (71+71*tanh(0.011*(GK-460))).*(1./(1+exp(0.02*(GK-460))));
    options = optimoptions('fsolve','Display','none');
    GK = fsolve(@(GK) (1/VGK)*(QGK*(GH-GK)-rKGE(GK)),GH,options);
    %GK = -0.008902 + 0.9999*GH + 0.0004381*QGK; %Fitted a plane to the solution for GK.
GL=(1/QGL)*(QGH*GH+rRBCU-QGB*GBC-QGK*GK-QGP*GPC);
rHGP=QGL*GL-QGA*GH-QGG*GG+rHGU;
    x(33,1)=GBC; x(34,1)=GBF; x(35,1)=GH; x(36,1)=GG;
    x(37,1)=GL; x(38,1)=GK; x(39,1)=GPC; x(40,1)=GPF;
%% Insulin basal values
    t2 = QIP.^2;
    t3 = QIP.*6.0e+1;
    t4 = VIPF.*1.7e+1;
    t5 = IPF.*QIP.*TIP.*3.0;
    t8 = IPF.*VIPF.*2.0e+1;
    t9 = IPF.*QIP.*-6.0e+1;
    t6 = -t4;
    t7 = IPF.*t3;
    t12 = t5+t8+t9;
    t10 = t3+t6;
    t11 = 1.0./t10;
    t13 = t11.*t12;
    t14 = -t13;
    outI = [t14;t14;t14;(t11.*(IPF.*t2.*-7.8e+2-IPF.*QIB.*QIP.*7.8e+2+IPF.*QIH.*QIP.*7.8e+2-IPF.*QIK.*QIP.*6.0e+2+IPF.*QIB.*VIPF.*2.6e+2-IPF.*QIH.*VIPF.*2.6e+2+IPF.*QIK.*VIPF.*2.0e+2+IPF.*QIP.*VIPF.*2.21e+2+IPF.*QIB.*QIP.*TIP.*3.9e+1-IPF.*QIH.*QIP.*TIP.*3.9e+1+IPF.*QIK.*QIP.*TIP.*3.0e+1))./(QIL.*1.3e+1);t13.*(-1.0e+1./1.3e+1);-t11.*(t5+t9+IPF.*t4);(t11.*(IPF.*t2.*-3.9e+3-IPF.*QIA.*QIP.*2.34e+3-IPF.*QIB.*QIP.*3.9e+3-IPF.*QIG.*QIP.*2.34e+3+IPF.*QIH.*QIP.*3.9e+3-IPF.*QIK.*QIP.*3.0e+3+IPF.*QIA.*VIPF.*7.8e+2+IPF.*QIB.*VIPF.*1.3e+3+IPF.*QIG.*VIPF.*7.8e+2-IPF.*QIH.*VIPF.*1.3e+3+IPF.*QIK.*VIPF.*1.0e+3+IPF.*QIP.*VIPF.*1.105e+3+IPF.*QIA.*QIP.*TIP.*1.17e+2+IPF.*QIB.*QIP.*TIP.*1.95e+2+IPF.*QIG.*QIP.*TIP.*1.17e+2-IPF.*QIH.*QIP.*TIP.*1.95e+2+IPF.*QIK.*QIP.*TIP.*1.5e+2))./3.9e+1];
    x(26,1)=outI(1); x(27,1)=outI(2); x(28,1)=outI(3); x(29,1)=outI(4);
    x(30,1)=outI(5); x(31,1)=outI(6); x(32,1)=IPF; rPIR=outI(7); 
%% Glucose absorption model:
    x(1,1)=0;
    x(2,1)=0;
    x(3,1)=0;
%% Metformin:
    x(4,1)=0; x(5,1)=0; x(6,1)=0; x(7,1)=0; 
    x(8,1)=0; x(9,1)=0;
%% Vildagliptin:
    x(10,1)=0; x(11,1)=0; x(12,1)=0; x(13,1)=0;
    x(14,1)=0; x(15,1)=0;
%% Physical activity:
    x(16,1)=0; x(17,1)=0;
%% Fast acting insulin:
    x(18,1)=0; x(19,1)=0;
%% Long acting inslin:
    x(20,1)=0; x(21,1)=0; x(23,1)=0;
%% Pancreas: 
    XG=x(35,1)^(3.27)/(1.32^3.27+5.93*x(35,1)^3.02);
    Pinft=XG^(1.11);
    Y=Pinft;
    R=XG;
    mpan=(kdmdpan+gammapan*Pinft)/(Kpan+((N1*Y).*(1./(1+exp(0.5*(XG-R))))+...
      (N1*Y+N2*(XG-R)).*(1./(1+exp(-0.5*(XG-R))))));
    S=N1*Y*mpan;
    x(23,1)=mpan; x(24,1)=Pinft; x(25,1)=XG;
%%  Glucagon:
    x(41,1)=1;
%%  GLP-1:
    x(42,1)=0; x(43,1)=0;

%% Basal rates: 
    x(44,1)=1;
    x(45,1)=0.0027;%0.5*(2.7*tanh(0.39*1)-1);
    x(46,1)=1;
    rates=[rPIR;rBGU;rRBCU;rGGU;rPGU;rHGP;rHGU];
    
%% Zero initial conditions:
x = [x;0;0;0;0;0;0;0;0;0;0;0;zeros(12,1);zeros(4,1)];
x(48) = 0.00000001;
end