function yinterp = ntrp78split(tinterp,t,y,h,f1,f6,f7,f8,f9,f10,f11,f12,f14,f15,f16,f17,idxNonNegative)
%NTRP78SPLIT  Interpolation helper function for ODE78.
%   YINTERP = NTRP78SPLIT(TINTERP,T,Y,TNEW,YNEW,H,F,IDX) uses data computed
%   in ODE78 to approximate the solution at time TINTERP. TINTERP may be a
%   scalar or a row vector. The arguments TNEW and YNEW do not affect the
%   computations. They are required for consistency of syntax with other
%   interpolation functions. Any values entered for TNEW and YNEW are
%   ignored.
%
%   IDX has indices of solution components that must be non-negative.
%   Negative YINTERP(IDX) are replaced with zeros and the derivative
%   YPINTERP(IDX) is set to zero.
%
%   See also ODE78, DEVAL.

%   Copyright 1984-2021 The MathWorks, Inc.

% Define the columns of coefficients of the 7th-order continuous extension
% of the Verner 8(7) "most efficient" pair. The weights are formed as 
% BI7*theta^7 + BI6*theta^6 + ... + BI2*theta^2 + BI1*theta, where 
% theta = (tinterp - t)/h. Since stages 2, 3, 4, 5, and 13 are not used,
% these rows in the the full coefficient matrix are zero. They are omitted
% here and the other rows shifted up. BI1 is not defined explicitly because
% it is [1;0;0;...;0].

BI2 = [-7.238550783576432811855355839508646327161;
    11.15330887588935170976376962782446833855;
    2.34875229807309355640904629061136935335;
    -1027.321675339240679090464776362465090654;
    1568.546608927281956416687915664731868885;
    -2000.882061921041961546811133479107090218;
    1496.620400693446268810344884971434468267;
    -16.41320775560933621675902845723196069900;
    -4.29672443178246482824254064733546854251;
    -20.41628069294821485579834313809132051248;
    16.53007184264271512356106095760699278945;
    -18.63064171313429626683549958846959067803];

BI3 = [26.00913483254676138219215542805486438340;
    -91.7609656398961659890179437322816238711;
    -11.6724894172018429369093778842231443146;
    9198.71432360760879019681406218311101879;
    -13995.38852541600542155322174511897930298;
    17864.36380347691630038038755096765127729;
    -13397.55405171476021512904990709508924800;
    147.6097045407002371315249807692915435608;
    38.6444746111678092366406218271498656093;
    153.5213232524836445391962375168798263930;
    -96.6861433615782065041742809436987893361;
    164.1994112280183092456176460821337125030];

BI4 = [-50.23684777762566731759165474184543812128;
    291.7074241722059450113911477530513089255;
    -3.339139076505928386509206543237093540;
    -33189.78048157363822223641020734287802492;
    50256.2124698102445419491620666726469821;
    -64205.1907515562863000297926577113695108;
    48323.5602199437493999696912750109765015;
    -535.719963714732106447158760197417632645;
    -140.3503471762808981414524290552248895548;
    -436.5502610211220460266289847121377276100;
    268.959934219531723149495873437076657635;
    -579.272256249540441494196462569641132906];

BI5 = [52.12072084601022449485077581012685809554;
    -430.4096692910862817449451677633631387823;
    94.885262249720610030798242337479596095;
    57750.0831348887181073584126028277545727;
    -86974.5128036219909523950692144595063700;
    111224.8489930378077126420609392735999202;
    -84051.4283423393032636942266780744607468;
    938.286247077820650371318861625025573381;
    246.3954669697502467443139611011701827640;
    598.214644262650861959065070073603792110
    -428.681909788964647271837835032326719249;
    980.198255708866731505258442280896479501];

BI6 = [-27.06472451211777193118825764262673140465;
    299.4531188198997479843407054776900024282;
    -143.071126583012024456409244370652716962;
    -47698.93315706261990169947144294597707756;
    71494.7977095997701213661747332399327008;
    -91509.3392102130338542605593697286718077;
    69399.8582111570893316100585838633124312;
    -779.438309639349328345148153897689081893;
    -205.8341686964167118696204191085878165880;
    -398.7823950071290897160364203878571043995;
    354.578231152433375494079868740183658991;
    -786.224179015513894176220583239056456901];

BI7 = [5.454547288952965694339504452480078562780;
    -79.78911199784015209705095616004766020335;
    61.0967097444217359754873031115590556707;
    14951.54365344033382142012769129774268946;
    -22324.57139433374168317029445568645401598;
    28594.46085938937782634638310955782423389;
    -21748.11815446623273761450332307272543593;
    245.4393970278627292916961100938952065362;
    65.44129872356201885836080588282812631205;
    104.0129692060648441002024406476025340187;
    -114.7001840640649599911246871588418008302;
    239.7294100413035911863764570341369884827];

yinterp = y(:,ones(size(tinterp)));
for k = 1:numel(tinterp)
    yinterp(:,k) = yinterp(:,k) + h*yinterpWSS((tinterp(k) - t)/h, ...
        BI2,BI3,BI4,BI5,BI6,BI7, ...
        f1,f6,f7,f8,f9,f10,f11,f12,f14,f15,f16,f17);
end

% Non-negative solution
if ~isempty(idxNonNegative)
    for k = idxNonNegative
        yinterp(k,:) = max(yinterp(k,:),0);
    end
end

%--------------------------------------------------------------------------

function sp = yinterpWSS(theta, ...
    BI2,BI3,BI4,BI5,BI6,BI7, ...
    f1,f6,f7,f8,f9,f10,f11,f12,f14,f15,f16,f17)
% Weighted sum of stages used to compute yinterp for
% theta = (tinterp - t)/h.
b = BI7;
b = b*theta + BI6;
b = b*theta + BI5;
b = b*theta + BI4;
b = b*theta + BI3;
b = b*theta + BI2;
% BI1 = [1;0;...;0] so we perform the last step
%     b = (b*theta + BI1)*theta;
% in slightly different fashion.
b = b*(theta*theta);
b(1) = b(1) + theta;
sp = b(1)*f1 + ...
    b(2)*f6 + ...
    b(3)*f7 + ...
    b(4)*f8 + ...
    b(5)*f9 + ...
    b(6)*f10 + ...
    b(7)*f11 + ...
    b(8)*f12 + ...
    b(9)*f14 + ...
    b(10)*f15 + ...
    b(11)*f16 + ...
    b(12)*f17;

%--------------------------------------------------------------------------