function [y,yt] = perday(obj,tpd,upd)
%This function can be used to repeat the same inputs per day for the entire
%simulation
%tpd: Times at which the inputs happens (in minuttes)
%upd: Input values at the specific times.
tday = 0:obj.dt:(24*60-1);
if(isempty(tpd))
    y=zeros(1,length(tday));
    yt=zeros(1,length(tday));
    y = repmat(y,[1 round(obj.Nd)]);
    yt = y>0;
else
    for k =1:length(tpd)
        [~,ind(k)] = min(abs(tday-tpd(k)));
    end
    y = zeros(1,length(tday));
    y(ind) = upd;
    y = repmat(y,[1 round(obj.Nd)]);
    yt = y>0;
end
Tend = length(obj.T);
if Tend>length(y)
y = [y zeros(1,Tend-length(y)-1)];
yt = [yt zeros(1,Tend-length(yt)-1)];
elseif Tend<length(y)
y = y(1:Tend-1);
yt = yt(1:Tend-1);
end
end