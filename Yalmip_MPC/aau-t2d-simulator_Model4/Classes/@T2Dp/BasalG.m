function BasalG = BasalG(GPC,QGA,QGB,QGG,QGH,QGK,QGL,QGP,TGB,TGP,VGBF,VGPF,rBGU,rGGU,rHGU,rKGE,rPGU,rRBCU)
% GH=GPC+rPGU/QGP;
% GK=GH;
% GBC=GH-rBGU/QGB;
% GG=GH-rGGU/QGG;
% GL=(1/QGL)*(QGA*GH+QGG*GG+rHGP-rHGU);
% GBF=GBC-rBGU*TGB/VGBF;
% GPF=GPC-rBGU*TGP/VGPF;
GPF=GPC-TGP*rPGU/VGPF;
GH=GPC+(VGPF/(QGP*TGP))*(GPC-GPF);
GK=GH;
GG=GH-rGGU/QGG;
GBC=GH-(1/QGB)*rBGU;
GBF=GBC-(TGB/VGBF)*rBGU;
GL=(1/QGL)*(QGH*GH+rRBCU-QGB*GBC-QGK*GK-QGP*GPC);
rHGP=QGL*GL-QGA*GH-QGG*GG+rHGU;
BasalG=[GBC GBF GH GG GL GK GPF rHGP];
end