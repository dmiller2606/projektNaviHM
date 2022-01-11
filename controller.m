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

uiwait(f)
arrobot_disconnect
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

