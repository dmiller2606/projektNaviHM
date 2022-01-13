clear all;
close all;

% Ziellienen koordinaten:
ziel = [10 -10; 10 10]

%  Init of the UDP Values
ip = "localhost";
portSelf = 9091;
portClient = 9090;

% Create UDP Object that handles messaging:
udpServer = udpport('LocalPort', portSelf);
configureTerminator(udpServer,"CR/LF");
while(true)
    uReceiver1Count = udpServer.NumBytesAvailable;
    if uReceiver1Count > 0
        data = read(udpServer,uReceiver1Count,"double");
        curLoc = [data(end-1) data(end)];
        disp(curLoc);
        if curLoc(1) >= ziel(1,1) - .5  && curLoc(1) <= ziel(2,1) + .5 && curLoc(2) >= ziel(1,2) - 1 && curLoc(2) <= ziel(2,2) + 1
            %disp("Goal");
            writeline(udpServer,"Goal!","127.0.0.1",portClient);
        end
    end
end