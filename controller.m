clear all;
close all;

ver

%% Aria Pfad und Neustart
% ==> notwendiger work around
% pfad hinzufuegen
addpath('C:\Daten_Alex\HM München (Uni)\5. Semester Kurse\Projekt Navigation\Materialien (1)\Materialien\Software\ARIA_2.9.1 (64-bit)_matlab_precompiled\ARIA_2.9.1 (64-bit)_matlab_precompiled');

% disconnet robot
arrobot_disconnect
% nochmals clear notwendig
clear all
% pfad hinzufuegen
addpath('C:\Daten_Alex\HM München (Uni)\5. Semester Kurse\Projekt Navigation\Materialien (1)\Materialien\Software\ARIA_2.9.1 (64-bit)_matlab_precompiled\ARIA_2.9.1 (64-bit)_matlab_precompiled');
% <==
aria_init -rh localhost -rrtp 8101
arrobot_connect
%% GUI Init
f = figure;
[I,map] = imread('img/leftArrow.png');
I = imresize(I,[100 100]);
I = ind2rgb(I,map);

global up
global down
global left
global right

% Down
down = uicontrol('Style','togglebutton','CData',flip(rot90(I)),...
    'Units','normalized','Position',[0.4 0.2 0.2 0.2],...
    'Callback',{@backward});
% Right
right = uicontrol('Style','togglebutton','CData',I,...
    'Units','normalized','Position',[0.6 0.4 0.2 0.2],...
    'Callback',{@rotateClock});
% up
up = uicontrol('Style','togglebutton','CData',rot90(I),...
    'Units','normalized','Position',[0.4 0.6 0.2 0.2],...
    'Callback',{@forward});
% left
left = uicontrol('Style','togglebutton','CData',flip(I,2),...
    'Units','normalized','Position',[0.2 0.4 0.2 0.2],...
    'Callback',{@rotateCounterClock});

% Koppenavi Schleife:
pG = poseGraph();
slam = lidarSLAM(20,5);
pre = [0 0 0];

XwandList = [];
YwandList = [];

while(size(findobj(f))>0)
   pause(0.5)
   ranges = [];
   pos = arrobot_getpose();
   pos(1) = pos(1)/1000;
   pos(2) = pos(2)/1000;
   pos(3)= pos(3)*(pi/180);

   angles = [90,50,30,10,350,330,310,270,270,230,210,190,170,150,130,90];
   senpos = [0.069, 0.136; 0.114, 0.119; 0.148, 0.078; 0.166 0.027; 0.166 -0.027; 0.148, -0.078;
            0.114, -0.119; 0.069, -0.136; -0.157, -0.136; 0.203, -0.119; -0.237, -0.078; 0.255, -0.027;
            -0.255, 0.027; -0.255, 0.027; 0.203, 0.119; 0.203, 0.119];

   for i = 0:15
      angles(i+1) = angles(i+1) * (pi/180);
      ranges(i+1) = arrobot_getsonarrange(i)/1000;
      %u = acos(dot([pos(1),pos(2)],senpos(i+1,:)));
      %w = (sqrt(pos(1)^2+pos(2)^2)*sqrt(senpos(i+1,1)^2+senpos(i+1,2)^2));
      %Win = acos(dot([pos(1),pos(2)],senpos(i+1,:))/(sqrt(pos(1).^2+pos(2).^2)*sqrt(senpos(i+1,1)^2+senpos(i+1,2)^2)));
      Xwand(i+1) = senpos(i+1,1)  + ranges(i+1) * cos(angles(i+1));
      Ywand(i+1) = senpos(i+1,2)  + ranges(i+1) * sin(angles(i+1));
   end
   
   XwandList = XwandList + Xwand(i+1);
   YwandList = YwandList + Ywand(i+1);

   scan = lidarScan([transpose(Xwand),transpose(Ywand)]);
   scan = removeInvalidData(scan,'Rangelimit',[0.01,4.9]);
   addScan(slam,scan,pos);
   addRelativePose(pG,pos);
   arrobot_setpose(0,0,0);
   subplot(3,3,1);
   show(pG);
end
% Disconnect
arrobot_disconnect

figure
show(slam);
%% Controlling Functions
    
function forward(source,eventdata)
    state = get(source,'Value');
    if state == 1
        arrobot_setvel(1000)
    else
        arrobot_setvel(0)
    end
end

function backward(source,eventdata)
        state = get(source,'Value');
    if state == 1
        arrobot_setvel(-1000)
    else
        arrobot_setvel(0)
    end
end

function rotateCounterClock(source,eventdata)
        state = get(source,'Value');
    if state == 1
        arrobot_setrotvel(30)
    else
        arrobot_setrotvel(0)
    end
end

function rotateClock(source,eventdata)
        state = get(source,'Value');
    if state == 1
        arrobot_setrotvel(-30)
    else
        arrobot_setrotvel(0)
    end
end

