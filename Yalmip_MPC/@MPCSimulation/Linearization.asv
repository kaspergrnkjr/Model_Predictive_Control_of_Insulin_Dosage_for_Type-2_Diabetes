function Linearization(obj, measurement)
%LINEARIZATION Summary of this function goes here
%   Detailed explanation goes here
    if(obj.meal)
        obj.dss = 0;
        obj.D1ss = obj.pd*1000/180.1559*obj.dss;
        obj.D2ss = obj.D1ss;
        obj.gss = measurement;
        obj.iss = (obj.p6 + obj.D2ss/(obj.pv*obj.pd) - obj.p5*obj.gss)/(obj.p4*obj.gss);
        obj.ipss = obj.iss - obj.p7*obj.gss;
        obj.iscss = obj.ipss;
        obj.uss = obj.iscss;
        obj.A = [-1/obj.p1 0 0 0 0 0;
            1/obj.p1 -1/obj.p1 0 0 0 0;
            0 obj.p3 -obj.p3 obj.p3*obj.p7 0 0;
            0 0 -obj.p4*obj.gss -(obj.p5 + obj.p4*(obj.iss)) 0 1/(obj.pd*obj.pv);
            0 0 0 0 -1/obj.pd 0;
            0 0 0 0 1/obj.pd -1/obj.pd];

        obj.xss = [obj.iscss obj.ipss obj.iss obj.gss obj.D1ss obj.D2ss];
    else
        obj.gss = measurement;
        obj.iss = (obj.p6 - obj.p5*obj.gss)/(obj.p4*obj.gss);
        obj.ipss = obj.iss - obj.p7*obj.gss;
        obj.iscss = obj.ipss;
        obj.uss = obj.iscss;
        obj.A = [-1/obj.p1 0 0 0 ;
                1/obj.p1 -1/obj.p1 0 0 ;
                0 obj.p3 -obj.p3 obj.p3*obj.p7 ;
                0 0 -obj.p4*obj.gss -(obj.p5 + obj.p4*(obj.iss))];

        obj.xss = [obj.iscss obj.ipss obj.iss obj.gss];
    end
end

