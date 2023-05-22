function saveStuffForSensitivityUpdate(obj, states, t)
    obj.x_yesterday = states;
    obj.t_yesterday = t;
    obj.meals_yesterday = obj.actualMealInfo;
    obj.A_yesterday = obj.A;
    obj.xss_yesterday = obj.xss;
    obj.uss_yesterday = obj.uss;
    obj.dss_yesterday = obj.dss;

end