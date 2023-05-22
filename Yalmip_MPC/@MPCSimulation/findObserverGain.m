function  findObserverGain(obj)

    if(obj.estimateMethod == 3)
       if(obj.meal)
            O = [obj.C ; obj.C*obj.A ; obj.C*obj.A^2 ; obj.C*obj.A^3 ; obj.C*obj.A^4 ; obj.C*obj.A^5];
            t6 = O\[0 0 0 0 0 1]';
            t5 = obj.A*t6;
            t4 = obj.A*t5;
            t3 = obj.A*t4;
            t2 = obj.A*t3;
            t1 = obj.A*t2;
            
            desiredObsPoles = eig(obj.A)'*1.2;% INSERT DESIRED POLES HERE
            T = [t1 t2 t3 t4 t5 t6];
        else
            O = [obj.C ; obj.C*obj.A ; obj.C*obj.A^2 ; obj.C*obj.A^3];
            desiredObsPoles = eig(obj.A)'*1.2;% INSERT DESIRED POLES HERE
            t4 = O\[0 0 0 1]';
            t3 = obj.A*t4;
            t2 = obj.A*t3;
            t1 = obj.A*t2;
            
            T = [t1 t2 t3 t4];
        end
        
        
        A_o = T\obj.A*T;
        A_o = A_o.*(abs(A_o) > 1e-12); % Setting the VERY small values to zero
        C_o = obj.C*T;
        C_o = C_o.*(abs(C_o) > 1e-12); % Setting the VERY small values to zero
        
        L_o = acker(A_o', -C_o', desiredObsPoles)';
        obj.L = (T*L_o); % Transformation from OCF domain to original domain
    
    end

end

