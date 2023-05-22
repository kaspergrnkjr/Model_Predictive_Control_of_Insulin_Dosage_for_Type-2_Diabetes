function FastActingType(obj,Type)
if Type == "Aspart" || Type == "Lispro" || Type == "Glulisine" 
%    pfa=0.054069054447099; rfa=0.002597782310193; qfa=0.000173616293493;
%     bfa=0.246024860053944; kclf =0.014782204029598;
% pfa=0.028129517387884; rfa=0.012466142678559; qfa=-0.000000009999994;
%     bfa=0.186831727336136; kclf =0.024106243478703;
%  pfa=0.028604104166292; rfa=0.032657236124532; qfa=-0.000000009999998;
%     bfa=0.368878169545229; kclf =0.024368855208263;
pfa=0.033304427073854; rfa=0.192838157600319; qfa=-0.000000009999983;
    bfa=0.350073112766538; kclf =0.031321989850181;
obj.Param.pfa=pfa; obj.Param.rfa=rfa; obj.Param.qfa=qfa;
obj.Param.bfa=bfa; obj.Param.kclf = kclf;
elseif Type == "Soluble" || Type == "Neutral"
pfa=0.007515131220953; rfa=0.006284304511320; qfa=0.029902465151402;
bfa=0.402665052055251; kclf =0.017217088399008;
obj.Param.pfa=pfa; obj.Param.rfa=rfa; obj.Param.qfa=qfa;
obj.Param.bfa=bfa; obj.Param.kclf = kclf;
else
    error('Type can only be: Aspart, Lispro, Glulisine, Soluble, Neutral');
end

