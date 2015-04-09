fclose(serial('COM10','Baudrate', 9600));
fclose(instrfindall);
clear all;
close all;


arduino = serial('COM10', 'BaudRate', 9600);
fopen(arduino);
i = 1;
figure;
hold on;
t(1) = 1;
y(1) = 1;
h = plot(t,y); 
while true
    t(i) = i;
    y(i) = fscanf(arduino, '%5f')
    set(h, 'xdata', t, 'ydata', y);
    pause(.001);
    i = i + 1;
end