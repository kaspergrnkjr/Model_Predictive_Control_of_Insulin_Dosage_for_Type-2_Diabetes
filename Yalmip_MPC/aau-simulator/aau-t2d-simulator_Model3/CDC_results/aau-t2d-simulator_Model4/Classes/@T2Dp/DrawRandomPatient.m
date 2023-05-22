function  Theta = DrawRandomPatient(obj,ThetaName,ThetaSet)
%Draw Patients randomly with a specified set of parameters:
%ThetaName is an M Cellarray of parameters' names (as strings) to be
%selected randomly.
%ThataS is the set of parameters provided as an M*L matrix where M
%is the number of parameters and L is the number of different
%patients. 
Tunepnumber = size(ThetaSet,2);
Csw = cumsum((1/Tunepnumber)*ones(1,Tunepnumber));
patient = find(rand<Csw,1);
Theta = ThetaSet(:,patient);
M = length(ThetaName); 
for k=1:M
    obj.Param.(ThetaName{k}) = Theta(k);
end
end