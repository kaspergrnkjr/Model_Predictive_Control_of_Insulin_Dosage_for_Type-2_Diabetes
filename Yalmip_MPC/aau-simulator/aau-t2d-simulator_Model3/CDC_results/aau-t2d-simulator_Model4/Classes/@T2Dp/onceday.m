function [y,yt] = onceday(obj,tpd,upd)
%This function can be used to repeat the same inputs per day for the entire
%simulation
%tpd: Times at which the inputs happens (in minuttes)
%upd: Input values at the specific times.
Nminv = obj.Nd*(obj.dt*24*60);
         Tv = 0:(Nminv-1);
if(isempty(tpd))
    y=zeros(1,length(Tv));
    yt=zeros(1,length(Tv));
    yt = y>0;
else
    for k =1:length(tpd)
        [~,ind(k)] = min(abs(Tv-tpd(k)));
    end
    y = zeros(1,length(Tv));
    y(ind) = upd;
    yt = y>0;
end
end