%Define a patient class 
classdef T2Dp < handle
   properties
   Param = paramClass
   %Initial values:
   %Glucose concentration in the periphery central compartment:
   GBPC0=10/0.0555; % in mg/dl
    
   %Insulin concentraion in the in periphery compartment: 
   IBPF0=1; % in mU/l
   
   %Basal rates (All basal rates except for rHGP and rPIR
   % since they are calculated from the others):
   brates=struct('rBGU',70,'rRBCU',10,'rGGU',20,'rPGU',35,'rHGU',20);
   %Simulation parameters:
        Nd = 4 %Number of simulation days
        dt = 1;%Simulation step sizes for the inputs (in minutes)
        %Nmin; %Number of minutes for the simulation
        %T; %Time vector in minutes for the inputs
        
        %If it is to simulate with diffusion of GH, the define the step
        %size for discretizing the diffusion:
        dtdif = 5; %5 minutes defualt

        Meals;% in grams of carbohydrates
        
        MealsH;% in grams of carbohydrates

        MealsM;% in grams of carbohydrates

        MealsL;% in grams of carbohydrates
        
        MealsvL;% in grams of carbohydrates
        %Long acting insulin:
        Ula;% in Units
        
        %Fast acting insulin:
        Ufa;% in Units
        
        %Heart rate difference from base (HRb):
        HRT;%HR difference times
        HRType = {}; %HR difference types as a cell array of functions in time
        
        %Metformin doses: 
        Um;% in mg
        
        %Vildagliptin:
        Uv;% in mmol

        %GLP-1 agonists
        
        %Exenatide
        UGLP1h2;
        %Semaglutide
        UGLP1h24;

        %Stress: 
        StressT;%Stress times
        StressType = {}; %Stress types as a cell array of functions in time

       %Simulated states:
       X=struct();
       %Simulated time:
       time;
       %Final simulation states:
       Xend;
       % Initial conditions if simulation in loop:
       X0sim = [];
       
       %Last Meal state:
       LastMeal = 0;
   end
   
   %Dependent properties:
   properties (Dependent)
       %Closed loop mode:
       CLmode;
       %Initial conditions:
       X0;
       %Basal values:
       basal;
       %Time of the simulation:
       T;
       %Number of data points in minutes:
       Nmin;
   end
   
   properties (Dependent,Hidden)
       %Dependent Inputs:
       MealD;
       MealDH;
       MealDM;
       MealDL;
       MealDvL;
       %Long acting insulin:
       UlaD;% in Units
        
       %Fast acting insulin:
       UfaD;% in Units
        
       %Heart rate difference from base (HRb):
       HRdD;% in bpm
        
       %Metformin doses: 
       UmD;% in mg
        
       %Vildagliptin:
       UvD;% in mg

       %GLP-1 agonists
        
       %Exenatide
       UGLP1h2D;
       %Semaglutide
       UGLP1h24D;
       
       %Heart rate difference from base (HRb):
       HRTD;%HR difference times
       HRTypeD; %HR difference types as a cell array of functions in time
       
       %Stress: 
       StressTD;%Stress times
       StressTypeD; %Stress types as a cell array of functions in time
   end
   
   methods
       %Constructor: 
       function obj = T2Dp(Param,GBPC0,IBPF0,brates,Nd,dt,dtdif)
            if(nargin>0)
                if not(isempty(Param)) 
                    obj.Param = Param;
                end
                if not(isempty(GBPC0))
                obj.GBPC0 = GBPC0;
                end
                if not(isempty(IBPF0))
                obj.IBPF0 = IBPF0;
                end
                if not(isempty(brates))
                obj.brates = brates;
                end
                if not(isempty(Nd))
                obj.Nd = Nd;
                end
                if not(isempty(dt))
                obj.dt = dt;
                end
                if not(isempty(dtdif))
                obj.dtdif = dtdif;
                end
            end
            %obj.brates = brates;
            
            %obj.Nmin = obj.Nd*(obj.dt*24*60);
            %obj.T = 0:(obj.Nmin-1);
            obj.Meals = perday(obj,[],[]);
            obj.MealsH = perday(obj,[],[]);
            obj.MealsM = perday(obj,[],[]);
            obj.MealsL = perday(obj,[],[]);
            obj.MealsvL = perday(obj,[],[]);

            obj.Ula = perday(obj,[],[]);
            obj.Ufa = perday(obj,[],[]);
            obj.Um = perday(obj,[],[]);
            obj.Uv = perday(obj,[],[]);
            obj.UGLP1h2 = perday(obj,[],[]);
            obj.UGLP1h24 = perday(obj,[],[]);
            obj.StressT = 0;
            obj.StressType = {@(t,tn) 0} ; 
            obj.HRT = 0;
            obj.HRType = {@(t,tn) 0};
       end
       
       %Set:
      function  set.Param(obj,Param)
         obj.Param = Param;
      end
      
      function  set.GBPC0(obj,GBPC0)
         obj.GBPC0 = GBPC0;
      end 
      
      function  set.IBPF0(obj,IBPF0)
         obj.IBPF0 = IBPF0;
      end
      
      function  set.brates(obj,brates)
         obj.brates = brates;
      end
      
      function  set.Nd(obj,Nd)
         obj.Nd = Nd;
      end
      
      function  set.dt(obj,dt)
         obj.dt = dt;
      end
      
      function  set.X0(obj,X0)
         obj.X0 = X0;
      end
      
      function  set.Meals(obj,Meals)
         obj.Meals = Meals;
      end

      function  set.MealsH(obj,MealsH)
         obj.MealsH = MealsH;
      end

      function  set.MealsM(obj,MealsM)
         obj.MealsM = MealsM;
      end

      function  set.MealsL(obj,MealsL)
         obj.MealsL = MealsL;
      end

      function  set.MealsvL(obj,MealsvL)
         obj.MealsvL = MealsvL;
      end
      
      function  set.Ula(obj,Ula)
         obj.Ula = Ula;
      end
      
      function  set.Ufa(obj,Ufa)
         obj.Ufa = Ufa;
      end
      
      function  set.Um(obj,Um)
         obj.Um = Um;
      end
      
      function  set.Uv(obj,Uv)
         obj.Uv = Uv;
      end

      function  set.UGLP1h2(obj,UGLP1h2)
         obj.UGLP1h2 = UGLP1h2;
      end

      function  set.UGLP1h24(obj,UGLP1h24)
         obj.UGLP1h24 = UGLP1h24;
      end
      
      function  set.HRT(obj,HRT)
         obj.HRT = HRT;
      end

      function  set.HRType(obj,HRType)
         obj.HRType = HRType;
      end
      
      function  set.StressT(obj,StressT)
         obj.StressT = StressT;
      end

      function  set.StressType(obj,StressType)
         obj.StressType = StressType;
      end
      
      function Nminv = get.Nmin(obj)
         Nminv = obj.Nd*(obj.dt*24*60);
      end
      
      function Tv = get.T(obj)
         Nminv = obj.Nd*(obj.dt*24*60);
         Tv = 0:(Nminv);
      end
      
%       function X0v = get.X0(obj)
%          [X0v,~,~] = BasaldefGCPFIPF(obj);
%       end
      
      function MealD = get.MealD(obj)
          if sum(obj.Meals) == 0
            MealD = zeros(1,floor(obj.Nd*24*60));
          else
              MealD = obj.Meals;
          end
      end

      function MealDH = get.MealDH(obj)
          if sum(obj.MealsH) == 0
            MealDH = zeros(1,floor(obj.Nd*24*60));
          else
              MealDH = obj.MealsH;
          end
      end

      function MealDM = get.MealDM(obj)
          if sum(obj.MealsM) == 0
            MealDM = zeros(1,floor(obj.Nd*24*60));
          else
              MealDM = obj.MealsM;
          end
      end

      function MealDL = get.MealDL(obj)
          if sum(obj.MealsL) == 0
            MealDL = zeros(1,floor(obj.Nd*24*60));
          else
              MealDL = obj.MealsL;
          end
      end

       function MealDvL = get.MealDvL(obj)
          if sum(obj.MealsvL) == 0
            MealDvL = zeros(1,floor(obj.Nd*24*60));
          else
              MealDvL = obj.MealsvL;
          end
      end
      
      function UlaD = get.UlaD(obj)
          if sum(obj.Ula) == 0
            UlaD = zeros(1,floor(obj.Nd*24*60));
          else
              UlaD = obj.Ula;
          end
      end
      
      function UfaD = get.UfaD(obj)
          if sum(obj.Ufa) == 0
            UfaD = zeros(1,floor(obj.Nd*24*60));
          else
              UfaD = obj.Ufa;
          end
      end

      function UGLP1h2D = get.UGLP1h2D(obj)
          if sum(obj.UGLP1h2) == 0
            UGLP1h2D = zeros(1,floor(obj.Nd*24*60));
          else
              UGLP1h2D = obj.UGLP1h2;
          end
      end
      
      function UGLP1h24D = get.UGLP1h24D(obj)
          if sum(obj.UGLP1h24) == 0
            UGLP1h24D = zeros(1,floor(obj.Nd*24*60));
          else
              UGLP1h24D = obj.UGLP1h24;
          end
      end
      
      function UmD = get.UmD(obj)
          if sum(obj.Um) == 0
            UmD = zeros(1,floor(obj.Nd*24*60));
          else
              UmD = obj.Um;
          end
      end
      
      function UvD = get.UvD(obj)
          if sum(obj.Uv) == 0
            UvD = zeros(1,floor(obj.Nd*24*60));
          else
              UvD = obj.Uv;
          end
      end
      
      function HRTD = get.HRTD(obj)
          if sum(obj.HRT) == 0
            HRTD = 0;
          else
              HRTD = obj.HRT;
          end
      end

      function HRTypeD = get.HRTypeD(obj)
          if isempty(obj.HRType) 
            obj.HRType = {@(t,tn) 0};
          else
              HRTypeD = obj.HRType;
          end
      end
      
      function StressTD = get.StressTD(obj)
          if sum(obj.StressT) == 0
            StressTD = 0;
          else
              StressTD = obj.StressT;
          end
      end

      function StressTypeD = get.StressTypeD(obj)
          if isempty(obj.StressType) 
            obj.StressType = {@(t,tn) 0};
          else
              StressTypeD = obj.StressType;
          end
      end

     function X0 = get.X0(obj)
         
          if isempty(obj.X0sim)
            X0 = BasaldefGCPFIPF(obj);
          else
            X0 = obj.X0sim;
          end
     end
      
     function CLmode = get.CLmode(obj)
         
          if isempty(obj.X0sim)
            CLmode = 0;
          else
            CLmode = 1;
          end
      end
      
      function basalv = get.basal(obj)
         [X0v,rates,SB] = BasaldefGCPFIPF(obj);
         basal0=struct();
         basal0.GPF=X0v(40,1); basal0.IPF=obj.IBPF0;...
         basal0.IL=X0v(29,1); basal0.GL=X0v(37,1);
         basal0.Gamma=X0v(41,1); basal0.SB=SB; basal0.GH=X0v(35,1); 
         basal0.IH=X0v(27,1);
         basal0.rPIR = rates(1);
         basal0.rBGU = rates(2);
         basal0.rRBCU = rates(3);
         basal0.rGGU = rates(4);
         basal0.rPGU = rates(5);
         basal0.rHGP = rates(6);
         basal0.rHGU = rates(7);
         basalv = basal0;
      end
       
       %To get the initial conditions in case of IPF and GPC given:
       [X0,rates,S]=BasaldefPF(obj)
       
       %Dynamics:
       dx=fode(obj,t, x, Dg, stressv, HRv, T, param, basal);


       J = jacobian_fode_pot(obj,t, x, Dg, stressv, HRv, T, param, basal);
       %Function to define rectanglar inputs: 
       y = rectf(obj,c,t1,t2,T);
        
       %Function to define trapazoidal inputs: 
       y = Rampf(obj,c,t1,t2,t3,t4,T)
        
       %Function to define inputs perday:
       [y,yt] = perday(obj,tpd,upd);

       %Function to define inputs perday:
       [y,yt] = onceday(obj,tpd,upd);
        
       %Simulation function (ode45):
       [X,time,Xsamp,Tsamp]=Simulation(obj,dTsamp);
       
       %Simulation with diffusion on GH function (ode45):
       [X,time]=Simulation_diffusion(obj);

       %Faster Simulation function but less stable (ode15s):
       [X,time]=Simulationfast(obj);
       
       %Simulation with controller: 
       [X,time] = SimCont(obj,cont,Tc);
       
       %Measurement for SMBG: 
       MeasS = SMBGmeas(obj,Measpoints,mode);
       
       %CGM measurements:
       [MeasC,ycal,Tcgm] = CGMmeas(obj,TSMBG,cgmdt,model);
       
       %Draw Patients randomly with a specified set of parameters:
       %ThetaName is an M Cellarray of parameters' names (as strings) to be
       %selected randomly.
       %ThataS is the set of parameters provided as an M*L matrix where M
       %is the number of parameters and L is the number of different
       %patients. 
       DrawRandomPatient(obj,ThetaName,ThetaSet);
       
       %Choose long acting insulin:
       LongActingType(obj,Type);
       
       %Choose fast acting insulin:
       FastActingType(obj,Type);
   end
   methods (Static)
   %Dynamics-Efficient version for long simulation but less parameters:
       dx=fodeeff_mex(t, x, Dg, param, basal);

   end
   
end
