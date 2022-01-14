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

%  Init of the UDP Values
ip = "localhost";
portSelf = 9090;
portServ = 9091;

% Create UDP Object that handles messaging:
udpClient = udpport('LocalPort', portSelf);
configureTerminator(udpClient,"CR/LF");

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

% Schleife scannen, fahren, mappen:
pG = poseGraph();
XW = [0];
YW = [0];
thetaAb = [0];

slam = lidarSLAM(20,5);
scans = {lidarScan(0,0)};
frame = 1;
angles = [90,50,30,10,350,330,310,270,270,230,210,190,170,150,130,90];
angles = angles .* (pi/180);
senpos = [0.069, 0.136; 0.114, 0.119; 0.148, 0.078; 0.166 0.027; 0.166 -0.027; 0.148, -0.078;
            0.114, -0.119; 0.069, -0.136; -0.157, -0.136; -0.203, -0.119; -0.237, -0.078; -0.255, -0.027;
            -0.255, 0.027; -0.237, 0.078; -0.203, 0.119; -0.157, 0.136];

curpos = [0 0 0];

while(size(findobj(f))>0)
   ranges = [];
   pos = arrobot_getpose();
   pos(1) = pos(1)/1000;
   pos(2) = pos(2)/1000;
   pos(3)= pos(3)*(pi/180);
   for i = 0:15
      ranges(i+1) = arrobot_getsonarrange(i)/1000;
      Xwand(i+1) = senpos(i+1,1) + ranges(i+1) * cos(angles(i+1));
      Ywand(i+1) = senpos(i+1,2) + ranges(i+1) * sin(angles(i+1));
   end

   % Occupancy Grid
   scan = lidarScan([transpose(Xwand),transpose(Ywand)]);
   scan = removeInvalidData(scan,'RangeLimits',[0.01 4.99]);
   scan = transformScan(scan,pos);
   scans{end + 1} = scan;
   addRelativePose(pG,pos);
   frame = frame + 1;
  
   % MapBuilding
   posesEST = nodeEstimates(pG);
   thetaAb = posesEST(frame,3);

   for t = 0:15
   if ranges(t+1) > 4.99
   continue
   end
   XwandS(t+1) =  (Xwand(t+1) * cos(thetaAb) - Ywand(t+1) * sin(thetaAb)) + posesEST(frame,1);
   YwandS(t+1) =  (Xwand(t+1) * sin(thetaAb) + Ywand(t+1) * cos(thetaAb)) + posesEST(frame,2);
   end 
   
   XW =  horzcat(XW,XwandS);
   YW =  horzcat(YW,YwandS); 

   % collison avoidance
   entf3 = arrobot_getsonarrange(3)/1000;
   entf4 = arrobot_getsonarrange(4)/1000;
   entf11 = arrobot_getsonarrange(11)/1000;
   entf12 = arrobot_getsonarrange(12)/1000;

   if entf3 < 0.9 && pos(1) > 0 
   arrobot_setvel(0)
   end

   if entf4 < 0.9 && pos(1) > 0
   arrobot_setvel(0)
   end

   if entf11 < 0.9 && pos(1) < 0 
   arrobot_setvel(0)
   end

   if entf12 < 0.9 && pos(1) < 0
   arrobot_setvel(0)
   end

   arrobot_setpose(0,0,0);
   subplot(3,3,1);
   plot(scan);
   pG = optimizePoseGraph(pG);
   frame = frame + 1;
   poses = nodeEstimates(pG);
   curpos = poses(end,:);
   % Send current position as cartesian coords to server:
   write(udpClient,curpos(1:2),"double","127.0.0.1",portServ);
   uReceiver1Count = udpClient.NumBytesAvailable;
   if uReceiver1Count > 0
       data = readline(udpClient);
       if data == "Goal!"
           disp("Goal!");
           break
       end
   end    
   pause(0.25);
end

% Disconnect
arrobot_disconnect

figure
pG = optimizePoseGraph(pG);
poses = nodeEstimates(pG);
map = buildMap(scans,poses,30,5);

% Plot
hold on;
axis equal;
plot(poses(:,1),poses(:,2),'-.');
plot(XW, YW, '.')
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
        arrobot_setrotvel(20)
    else
        arrobot_setrotvel(0)
    end
end

function rotateClock(source,eventdata)
        state = get(source,'Value');
    if state == 1
        arrobot_setrotvel(-20)
    else
        arrobot_setrotvel(0)
    end
end

