function BasalI = BasalI(IPF,QIA,QIB,QIG,QIH,QIK,QIL,QIP,TIP,VIPF)
%BASALI
%    BASALI = BASALI(IPF,QIA,QIB,QIG,QIH,QIK,QIL,QIP,TIP,VIPF)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    24-Mar-2020 03:08:12


t2 = QIP.^2;
t3 = QIP.*6.0e+1;
t4 = VIPF.*1.7e+1;
t5 = IPF.*QIP.*TIP.*3.0;
t8 = IPF.*VIPF.*2.0e+1;
t9 = IPF.*QIP.*-6.0e+1;
t6 = -t4;
t7 = IPF.*t3;
t12 = t5+t8+t9;
t10 = t3+t6;
t11 = 1.0./t10;
t13 = t11.*t12;
t14 = -t13;
BasalI = [t14;t14;t14;(t11.*(IPF.*t2.*-7.8e+2-IPF.*QIB.*QIP.*7.8e+2+IPF.*QIH.*QIP.*7.8e+2-IPF.*QIK.*QIP.*6.0e+2+IPF.*QIB.*VIPF.*2.6e+2-IPF.*QIH.*VIPF.*2.6e+2+IPF.*QIK.*VIPF.*2.0e+2+IPF.*QIP.*VIPF.*2.21e+2+IPF.*QIB.*QIP.*TIP.*3.9e+1-IPF.*QIH.*QIP.*TIP.*3.9e+1+IPF.*QIK.*QIP.*TIP.*3.0e+1))./(QIL.*1.3e+1);t13.*(-1.0e+1./1.3e+1);-t11.*(t5+t9+IPF.*t4);(t11.*(IPF.*t2.*-3.9e+3-IPF.*QIA.*QIP.*2.34e+3-IPF.*QIB.*QIP.*3.9e+3-IPF.*QIG.*QIP.*2.34e+3+IPF.*QIH.*QIP.*3.9e+3-IPF.*QIK.*QIP.*3.0e+3+IPF.*QIA.*VIPF.*7.8e+2+IPF.*QIB.*VIPF.*1.3e+3+IPF.*QIG.*VIPF.*7.8e+2-IPF.*QIH.*VIPF.*1.3e+3+IPF.*QIK.*VIPF.*1.0e+3+IPF.*QIP.*VIPF.*1.105e+3+IPF.*QIA.*QIP.*TIP.*1.17e+2+IPF.*QIB.*QIP.*TIP.*1.95e+2+IPF.*QIG.*QIP.*TIP.*1.17e+2-IPF.*QIH.*QIP.*TIP.*1.95e+2+IPF.*QIK.*QIP.*TIP.*1.5e+2))./3.9e+1];
