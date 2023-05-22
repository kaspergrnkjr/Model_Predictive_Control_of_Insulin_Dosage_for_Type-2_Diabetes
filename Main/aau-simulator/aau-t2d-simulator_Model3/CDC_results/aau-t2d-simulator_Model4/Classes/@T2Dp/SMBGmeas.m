function Meas = SMBGmeas(obj,Measpoints,mode)
%SMBGmeas returns SMBG measurements specified at time points Measpoints of
%the simulation. 
K = length(Measpoints); 
id = ones(1,K);
for k=1:K
    [~,id(k)] = min(abs(obj.time-Measpoints(k)));
end
if mode == "simple" 
Meas = (1+0.01*obj.Param.sigsmbg*rand(size(obj.X.GH(id)))).*obj.X.GH(id);
elseif mode =="detailed"
    Meas = obj.X.GH(id);
    zeta1 = -5.37;
    w1 = 9.86;
    alpha1 = 2.72;
    zeta2 = -3.83;
    w2 = 13.17;
    alpha2 = 1.41;
    d1 = alpha1/sqrt(1-alpha1);
    d2 =alpha2/sqrt(1-alpha2);
    
    Meas(Meas<=75) = Meas(Meas<=75) + zeta2 + w2*(d2*abs(randn(size(Meas(Meas<=75))))+randn(size(Meas(Meas<=75)))*sqrt(1 - d2*d2));
    Meas(Meas>75) = Meas(Meas>75) + (zeta1 + w1*(d1*abs(randn(size(Meas(Meas>75))))+randn(size(Meas(Meas>75)))*sqrt(1 - d1*d1))).*Meas(Meas>75);
end
end