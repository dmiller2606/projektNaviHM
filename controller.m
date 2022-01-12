clear all;
close all;

ver

%% Aria Pfad und Neustart
% ==> notwendiger work around
% pfad hinzufuegen
addpath('C:\Users\dmill\Desktop\Projekt Navi\Materialien\Software\ARIA_2.9.1 (64-bit)_matlab_precompiled');

% disconnet robot
arrobot_disconnect
% nochmals clear notwendig
clear all
% pfad hinzufuegen
addpath('C:\Users\dmill\Desktop\Projekt Navi\Materialien\Software\ARIA_2.9.1 (64-bit)_matlab_precompiled');
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

% Koppelnavi Schleife:
pG = poseGraph();
scans = {lidarScan(0,0)};
frame = 1;
angles = [90,50,30,10,350,330,310,270,270,230,210,190,170,150,130,90];
senpos = [0.069, 0.136; 0.114, 0.119; 0.148, 0.078; 0.166 0.027; 0.166 -0.027; 0.148, -0.078;
            0.114, -0.119; 0.069, -0.136; -0.157, -0.136; -0.203, -0.119; -0.237, -0.078; -0.255, -0.027;
            -0.255, 0.027; -0.255, 0.078; -0.203, 0.119; -0.157, 0.136];
while(size(findobj(f))>0)
   ranges = [];
   pos = arrobot_getpose();
   pos(1) = pos(1)/1000;
   pos(2) = pos(2)/1000;
   pos(3)= pos(3)*(pi/180);
   for i = 0:15
      angles(i+1) = angles(i+1) * (pi/180);
      ranges(i+1) = arrobot_getsonarrange(i)/1000;
      Xwand(i+1) = senpos(i+1,1) + ranges(i+1) * cos(angles(i+1));
      Ywand(i+1) = senpos(i+1,2) + ranges(i+1) * sin(angles(i+1));
   end
   scan = lidarScan([transpose(Xwand),transpose(Ywand)]);
   scan = removeInvalidData(scan,'RangeLimits',[0.01 4.99]);
   scan = transformScan(scan,pos);
   scans{end + 1} = scan;
   addRelativePose(pG,pos);
   arrobot_setpose(0,0,0);
   subplot(3,3,1);
   plot(scan);
   pG = optimizePoseGraph(pG);
   frame = frame + 1;
   pause(0.25);
end
% Disconnect
arrobot_disconnect
figure
%addRelativePose(pG,pos,[1 0 0 1 0 1],frame,1);
pG = optimizePoseGraph(pG);
poses = nodeEstimates(pG);
map = buildMap(scans,poses,10,5);
show(pG);
figure
show(map);

%% Controlling Functions
    
function forward(source,eventdata)
    state = get(source,'Value');
    if state == 1
        arrobot_setvel(500)
    else
        arrobot_setvel(0)
    end
end

function backward(source,eventdata)
        state = get(source,'Value');
    if state == 1
        arrobot_setvel(-500)
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

