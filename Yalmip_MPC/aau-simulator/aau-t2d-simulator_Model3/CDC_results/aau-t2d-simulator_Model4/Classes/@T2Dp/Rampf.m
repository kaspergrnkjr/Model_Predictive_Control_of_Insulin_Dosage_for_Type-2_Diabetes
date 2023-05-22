%Ramp up Const Ramp down signal
function y = Rampf(obj,c,t1,t2,t3,t4,T)

[~,t1ind] = min(abs(T-t1));
[~,t2ind] = min(abs(T-t2));
[~,t3ind] = min(abs(T-t3));
[~,t4ind] = min(abs(T-t4));
y=zeros(1,length(T));
y(T<=t1) = 0;
y(t1<=T & T<=t2) = c/(t2-t1)*(T(t1ind:t2ind)-t1);
y(t2<=T & T<=t3) = c;
y(t3<=T & T<=t4) = -c/(t4-t3)*(T(t3ind:t4ind)-t4);

end