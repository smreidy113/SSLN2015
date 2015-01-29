close all;

res = 10;
size = 150;

distances = [-1 20 20 -1 -1 30 25 40 40 -1 45 40];

%% DRAW THE ROBOT

lowerbase = 30; % cm
upperbase = 50; % cm
height = 70; % cm

axis([-size/2 size/2 -size/2 size/2]);

hold on;
plot([-lowerbase/2, lowerbase/2], [-height/2, -height/2], 'r-', 'Linewidth', 5);
plot([-upperbase/2, upperbase/2], [height/2, height/2], 'r-', 'Linewidth', 5);
plot([-upperbase/2, -lowerbase/2], [height/2, -height/2], 'r-', 'Linewidth', 5);
plot([upperbase/2, lowerbase/2], [height/2, -height/2], 'r-', 'Linewidth', 5);