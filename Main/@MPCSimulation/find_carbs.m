function carbs = find_carbs(obj, t)

% find_carbs    Find the wanted carbs 
%
% This is done based on mealMeans and mealStd
% carbs = find_Carbs(t,states,dt). t is the given time, states is the
% states of the person and dt is the discrete time interval length
carbs = 0;
meanCarbs = 0;
boolVector = zeros(4,1);

if(round(mod(t,1), 10) == 0)
    if(obj.stochasticMeals)
        boolVector = unifrnd(0,1,4,1);
        boolVector = boolVector < obj.mealAdherence;
        obj.actualMealInfo = normrnd(obj.mealMeans, obj.mealStd);
        while(sum(obj.actualMealInfo(:,2))<obj.carbLowerBound)
            obj.actualMealInfo = normrnd(obj.mealMeans, obj.mealStd);
        end
        while(boolVector'*obj.actualMealInfo(:,2)<obj.carbLowerBound)
            boolVector = unifrnd(0,1,4,1);
            boolVector = boolVector < obj.mealAdherence;
        end
        obj.actualMealInfo(:,2) = obj.actualMealInfo(:,2).* boolVector;
    else
        obj.actualMealInfo = obj.mealMeans;
    end
end
for i = 1:1:size(obj.actualMealInfo, 1)
    if(round(mod(t,1),10) <= round(obj.actualMealInfo(i, 1),10) && round(mod(t + obj.timeStep,1),10)  > round(obj.actualMealInfo(i, 1),10))
        carbs = obj.actualMealInfo(i, 2);
        meanCarbs = obj.mealMeans(i, 2);
    end

end


carbs = carbs/(obj.timeStep);
meanCarbs = meanCarbs/(obj.timeStep);
obj.lastMeal = carbs;
obj.meanCarbs = meanCarbs;


end