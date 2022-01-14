clear all;
close all;

ver

%% Aria Pfad und Neustart
% ==> notwendiger work around
% pfad hinzufuegen
addpath('C:\1HM\Projekt Navigation\Materialien\Software\ARIA_2.9.1 (64-bit)_matlab_precompiled');

% disconnet robot
arrobot_disconnect
% nochmals clear notwendig
clear all
% pfad hinzufuegen
addpath('C:\1HM\Projekt Navigation\Materialien\Software\ARIA_2.9.1 (64-bit)_matlab_precompiled');
% <==
aria_init -rh localhost -rrtp 8101
arrobot_connect

% P = [10500 0;
%     10500 6600;
%     0     6600;
%     0     0];

P = [2152 0;	
2153 -1484;	
2800 -1529;	
4314 -1637;	
4427 152;	
6570 154;	
6270 -2924;	
8214 -3265;	
8741 -2345;	
6537 -2116;	
6703 296;	
10950 302;	
10682 -3105;	
11480 -3076;	
11624 -2078;	
10951 -1732;	
10906 279;	
12645	74;
14909	-3249;
16196	-2373;
14830	-644;
13727	-529;
13695	1435;
15849	1441;
15551	4196;
12836	4206;
13007	4810;
14766	4693;
15045	7449;
14587	8569;
14550	9822;
16616	9610;
23348	10109;
24352	8073;
24484	4751;
28965	4522;
28558   1551;
28670   680;
28243   -1057;
24170   -505;
24157   3172;
26407   3023;
26425   1864;
24462   982;
24190   -2966;
28387   -3100;
28609   -909;
%28504   138;
28639   1408;
29955   1318;
29793   -1698;
30459   -3182;
33915   -2718;
33574   -834;
30054   -849;
30036   1820;
33010   1307;
37634   1809;
40320   5273;
42371   5045;
41812   2376;
43488   1338;
46544   7170;
43326   10003;
41263   8132;
38177   8447;
36217   8308;
34088   8449;
33660   5577;
37221   5956;
37619   3725;
33228   3102;
31148   3769;
29406   3768;
29536   7595;
28434   7478;
26826   7588;
26152   10274;
18248   10537;
17113   10775;
12308   10261;
10932   10336;
11298   6900;
11628   6907;
13021   6780;
11140   7035;
8369    7278;
5523    6917;
5077    5742;
8064    3962;
6302    2306;
2586    2617;
3244    5100;
5051    5729;
5239    6969;
3544    7102;
3525    8033;
3545    7047;
-80    6969;
-100     10265;
-2053   10215;
-1537   9331;
-517    9045;
-90    8301;
-110    6630;
-1848   6640;
-1910   3774;
-154    3619;
-400    1293;
-2048   1482;
-3683   487;
-2672   137;
-1996   1260;
-306    1337;
-237    -1567;
-1475   -3116;
-3267   -2844;
-2421   -1529;
-300    -1431;
-196    -12;
];


anzPktAbgef = 1;
t = 0;

%%Evtl. Todo
% P = [200 6500;
%     10500 6500;
%     10500 0;
%     0 0];

%Geschwindigkeitseinstellung und Losfahren des Roboters
arrobot_setvel(400)

%While-Schleife, die den gesamten Pfad-Abfahrprozess aktiv haelt, bis der
%letzte Pfadpunkt erreicht ist
while anzPktAbgef < size(P,1)+1

    pause(0.25)
    curPosX = arrobot_getx
    curPosY = arrobot_gety

    %Positionsvergleich zwischen aktueller Position (curPos = current Position)
    %und Zielposition
    %Wenn der Zielpunkt um maximal 250mm in jede Richtung verfehlt wird, gilt
    %der Punkt als abgefahren
    if curPosX < P(anzPktAbgef,1)+250 && curPosX >= P(anzPktAbgef,1)-250 && curPosY <= P(anzPktAbgef,2)+250 && curPosY >= P(anzPktAbgef,2)-250
        arrobot_setvel(0)
        pause(5)
        anzPktAbgef = anzPktAbgef + 1;

        %Abbruchbedingung für while-Schleife. Sorgt dafuer, dass am Ende des Pfads
        %korrekt beendet wird
        if size(P,1) < anzPktAbgef
            break;
        end

        %%Rotation
        %Berechnung der Koordinatenunterschiede delta aus aktueller Position und
        %naechstem Pfadpunkt
        deltaX = P(anzPktAbgef,1) - arrobot_getx
        deltaY = P(anzPktAbgef,2) - arrobot_gety

        %Fallunterscheidung Quadranten
        if deltaY > 0 && deltaX > 0
            t = 0;
        elseif deltaY > 0 && deltaX < 0
            t = 180;
        elseif deltaY < 0 && deltaX < 0
            t = 180;
        elseif deltaY < 0 && deltaX > 0
            t = 360;
        end

        % if deltaX == 0 || deltaY == 0
        %     phiGrad = 90;
        % else
        %Errechnung Drehwinkel in Grad mit Hilfe der Koordinatenunterschiede delta
        phiGrad = atan(deltaY/deltaX) * (180/pi) + t - arrobot_getth
        % end
        %Ausfuehren der Drehung des Roboters auf naechsten Punkt, pause(9)
        %evtl. verlaengern bei groesseren Drehwinkeln als 90 Grad
        arrobot_setdeltaheading(phiGrad)
        pause(9)
        
        %Geschwindigkeitseinstellung und Losfahren des Roboters nach
        %Drehung
        arrobot_setvel(400)
    end
end
% Disconnect
arrobot_disconnect
