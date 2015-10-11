% A = csvread('C:\Users\Filipe\Documents\visual studio 2013\Projects\MRLS\MRLS\log_file.txt');
% TS = A(:,1);
% X = A(:,9);
% Y = A(:,10);
% marker = A(:,11);
% TIME = cumsum(TS);
% figure
% plot(TIME,X,'r',TIME,Y,'g')
% title('Sensor Fusion')
% xlabel('Time(s)')
% ylabel('Distance(cm)')
% legend('x´','y´')
% grid on;
% hold on;
% % for i= 1:size(marker)
% %     if marker(i) == 1
% %         yL = get(gca, 'YLim');
% %         l1 = line([TIME(i) TIME(i)], yL,  'Color', 'b');
% %         l1.Color(4) = 0.4;
% %         hold on;
% %     end
% % end;
% 
% B = csvread('C:\Users\Filipe\Documents\visual studio 2013\Projects\MRLS\MRLS\marker_log_file.txt');
% TS = B(:,1);
% X = B(:,2);
% Y = B(:,3);
% TIME = cumsum(TS);
% figure
% plot(TIME,X,'r',TIME,Y,'g')
% title('Marker Only')
% xlabel('Time(s)')
% ylabel('Distance(cm)')
% legend('x´','y´')
% grid on

A = csvread('C:\Users\Filipe\Documents\visual studio 2013\Projects\MRLS\MRLS\testLogFile.txt');%log_file.txt');
TS = A(:,1);
X = A(:,2);
X = X./100;
Y = A(:,3);
Y = Y./100;
UX = A(:,11);
UY = A(:,12);
RawX = A(:,29);
RawY = A(:,30);
TIME = cumsum(TS);
figure
subplot(2,2,1)
plot(TIME,X,'r',TIME,Y,'g')
title('Modified ACCEL')
xlabel('Time(s)')
%ylabel('Distance(cm)')
legend('x´','y´')
grid on;
subplot(2,2,2)
plot(TIME,UX,'r',TIME,UY,'g')
title('Real Accel')
xlabel('Time(s)')
%ylabel('Distance(cm)')
legend('x´','y´')
grid on;
% subplot(3,1,3)
% plot(TIME,RawX,'r',TIME,RawY,'g')
% title('Real Accel')
% xlabel('Time(s)')
% ylabel('Distance(cm)')
% legend('x´','y´')
% grid on;