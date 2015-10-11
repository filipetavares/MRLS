function DebugPlot
% A = csvread('C:\Users\Filipe\Documents\visual studio 2013\Projects\MRLS\MRLS\log_file.txt');
% TS = A(:,1);
% AX = A(:,2);
% AY = A(:,3);
% AZ = A(:,4);
% VX = A(:,5);
% VY = A(:,6);
% PX = A(:,7);
% PY = A(:,8);
% PPX = A(:,9);
% PPY = A(:,10);
% TIME = cumsum(TS);
% figure
% subplot(2,2,1)
% plot(TIME,AY,'r',TIME,AZ,'g')
% title('Acceleration')
% xlabel('Time(s)')
% legend('Ax','Ay')
% grid on
% subplot(2,2,2)
% plot(TIME,VX,'r',TIME,VY,'g')
% title('Velocity')
% xlabel('Time(s)')
% legend('Vx','Vy')
% grid on
% subplot(2,2,3)
% plot(TIME,PX,'r',TIME,PY,'g')%,TIME,PPX,'r--',TIME,PPY,'g--')
% title('Position(cm)')
% xlabel('Time(s)')
% legend('Px','Py')%,'PPx', 'PPy')
% grid on
fig = figure();
A = csvread('C:\Users\Filipe\Documents\visual studio 2013\Projects\MRLS\MRLS\log_file.txt');
%A = csvread('C:\Users\Filipe\Documents\visual studio 2013\Projects\MRLS\MRLS\testLogFile.txt')
TS = A(:,1);
AX = A(:,2);
AY = A(:,3);
AZ = A(:,4);
VX = A(:,5);
VY = A(:,6);
PX = A(:,7);
PY = A(:,8);
PPX = A(:,9);
PPY = A(:,10);
VO = A (:,15);
O = A(:,16);
MARKER = A(:,21);
TIME = cumsum(TS);
subplot(2,2,1)
plot(TIME,AX,'r',TIME,AY,'g')
title('Acceleration')
xlabel('Time(s)')
%legend('Ax','Ay')
grid on
subplot(2,2,2)
plot(TIME,VX,'r',TIME,VY,'g')
title('Velocity')
xlabel('Time(s)')
 %legend('Vx','Vy')
grid on
subplot(2,2,3)
plot(TIME,PX,'r',TIME,PY,'g')%,TIME,PPX,'r--',TIME,PPY,'g--')
title('Position(cm)')
xlabel('Time(s)')
%legend('Px','Py')%,'PPx', 'PPy')
grid on
subplot(2,2,3)
plot(TIME,PX,'r',TIME,PY,'g')%,TIME,PPX,'r--',TIME,PPY,'g--')
title('Position(cm)')
xlabel('Time(s)')
%legend('Px','Py')%,'PPx', 'PPy')
grid on


dcm_obj = datacursormode(fig);
set(dcm_obj,'UpdateFcn',{@myupdatefcn,PX,PY})
function txt = myupdatefcn(~,event_obj,PX,PY)
% Customizes text of data tips
pos = get(event_obj,'Position');
I = get(event_obj, 'DataIndex');
txt = {['TIME: ',num2str(pos(1))],...
       ['X: ',num2str(PX(I))],...
       ['Y: ',num2str(PY(I))]};
       %['T: ',num2str(t(I))]};
%     figure
%     subplot(2,2,1)
%     plot(TIME,VO,'r')
%     title('Angular Velocity')
%     xlabel('Time(s)')
%     %legend('Ax','Ay')
%     grid on
%     subplot(2,2,2)
%     plot(TIME,0,'r')
%     title('Angle')
%     xlabel('Time(s)')
%     %legend('Vx','Vy')
%     grid on
    
