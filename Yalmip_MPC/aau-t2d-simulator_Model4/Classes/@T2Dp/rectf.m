%Square function:
function y = rectf(obj,c,t1,t2,T)
%c is the rectangle constant value 
%t1 is the begining of the rectangle
%t2 is the end of the rectangle
%T is the time vector of which t1 and t2 belongs
y = zeros(1,length(T));
[~,ind1] = min(abs(T-t1));
[~,ind2] = min(abs(T-t2));
y(ind1:ind2) = c; 
end