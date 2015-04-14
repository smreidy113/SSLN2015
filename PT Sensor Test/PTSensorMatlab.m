try
    clear
    clc
    arduino = serial('COM6', 'BaudRate', 9600);
    fopen(arduino);
    
    formatSpec = '%c';
    
    i = 1;
    t(1) = 1;
    bottomL(1) = 0; % pot
    bottomR(1) = 0; % pot
    backRL(1) = 0; % pot
    backRM(1) = 0; % pot
    backRR(1) = 0; % pot
    backLM(1) = 0; % pot
    backLL(1) = 0; % pot
    backLR(1) = 0; % pot
    frontLL(1) = 0; % pot
    frontLM(1) = 0; % pot
    frontLR(1) = 0; % pot
    frontRL(1) = 0; % pot
    frontRM(1) = 0; % pot
    frontRR(1) = 0; % pot
    
    
    subplot(5,1,1)
    hold on
    a = plot(t,frontLL,'b-','LineWidth',2,'MarkerSize',8);
    b = plot(t,frontLM,'r-','LineWidth',2,'MarkerSize',8);
    c = plot(t,frontLR,'g-','LineWidth',2,'MarkerSize',8);
    hold off
    xlabel('number of data points')
    ylabel('front left')
    ylim([0 100])
    grid on
    
    subplot(5,1,2)
    hold on
    d = plot(t,frontRL,'b-','LineWidth',2,'MarkerSize',8);
    e = plot(t,frontRM,'r-','LineWidth',2,'MarkerSize',8);
    f = plot(t,frontRR,'g-','LineWidth',2,'MarkerSize',8);
    hold off
    xlabel('number of data points')
    ylabel('front right')
    ylim([0 100])
    grid on
    
    subplot(5,1,3)
    hold on
    g = plot(t,backLL,'b-','LineWidth',2,'MarkerSize',8);
    h = plot(t,backLM,'r-','LineWidth',2,'MarkerSize',8);
    p = plot(t,backLR,'g-','LineWidth',2,'MarkerSize',8);
    hold off
    xlabel('number of data points')
    ylabel('back left')
    ylim([0 100])
    grid on
    
    subplot(5,1,4)
    hold on
    j = plot(t,backRL,'b-','LineWidth',2,'MarkerSize',8);
    k = plot(t,backRM,'r-','LineWidth',2,'MarkerSize',8);
    l = plot(t,backRR,'g-','LineWidth',2,'MarkerSize',8);
    hold off
    xlabel('number of data points')
    ylabel('back right')
    ylim([0 100])
    grid on
    
    subplot(5,1,5)
    hold on
    m = plot(t,bottomR,'b-','LineWidth',2,'MarkerSize',8);
    n = plot(t,bottomL,'r-','LineWidth',2,'MarkerSize',8);
    hold off
    xlabel('number of data points')
    ylabel('bottom')
    ylim([0 100])
    grid on
    
    while true
        t(i) = i;
        new_data = str2double(strsplit(fscanf(arduino, formatSpec),'\t'));
        new_data = new_data(2:end-1);
        bottomR(i) = new_data(1); % pot 1 confirmed
        bottomL(i) = new_data(2); % pot 0 confirmed
        backRL(i) = new_data(7); % pot 2 confirmed
        backRM(i) = new_data(8); % pot 2
        backRR(i) = new_data(6); % pot 2 confirmed
        backLM(i) = new_data(5); % pot  confirmed
        backLL(i) = new_data(3); % pot  confirmed
        backLR(i) = new_data(4); % pot  confirmed
        frontLL(i) = new_data(9);
        frontLM(i) = new_data(10);
        frontLR(i) = new_data(11);
        frontRL(i) = new_data(14); % pot  confirmed
        frontRM(i) = new_data(13);
        frontRR(i) = new_data(12); % pot
        
        set(a, 'xdata', t, 'ydata', frontLL);
        set(b, 'xdata', t, 'ydata', frontLM);
        set(c, 'xdata', t, 'ydata', frontLR);
        set(d, 'xdata', t, 'ydata', frontRL);
        set(e, 'xdata', t, 'ydata', frontRM);
        set(f, 'xdata', t, 'ydata', frontRR);
        set(g, 'xdata', t, 'ydata', backLL);
        set(h, 'xdata', t, 'ydata', backLM);
        set(p, 'xdata', t, 'ydata', backLR);
        set(j, 'xdata', t, 'ydata', backRL);
        set(k, 'xdata', t, 'ydata', backRM);
        set(l, 'xdata', t, 'ydata', backRR);
        set(m, 'xdata', t, 'ydata', bottomR);
        set(n, 'xdata', t, 'ydata', bottomL);
        pause(.0001);
        i = i + 1;
    end
    
catch
    
    fclose(serial('COM6','Baudrate', 9600));
    fclose(instrfindall);
    
end