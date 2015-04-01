close all;

res = 10;
size = 150;

sensorpositions = [[-30 -6 -1 0];
                   [-30 -2 -.707 .707];
                   [-30 -2 0 1];
                   [30 -2 0 1];
                   [30 -2 .707 .707];
                   [30 -6 1 0];
                   [20 -12 0 -1];
                   [-20 -12 0 -1];
                   
                   [-5 8 -1 0];
                    [5 8 1 0]
                    [0 13 0 1];
                    [0 3 0 -1]];
                
                
objects = [[40 40 3];
           [-30 -20 10]];

distances = [-1 20 20 -1 -1 30 25 40 40 -1 45 40];

%% DRAW THE ROBOT

lowerbase = 60;
lowerheight = 10;
upperbase = 10;
upperheight = 10;
distbetween = 5;

overallbase = max(lowerbase, upperbase) + 10;
overallheight = lowerheight + distbetween + upperheight + 10;

lowercenter = -overallheight/2 + 5 + lowerheight/2;
uppercenter = overallheight/2 - 5 - upperheight/2;

axis([-size/2 size/2 -size/2 size/2]);
axis square;

hold on;
grid on;
plot([-overallbase/2, overallbase/2], [-overallheight/2, -overallheight/2], 'g-', 'Linewidth', 5);
plot([overallbase/2, overallbase/2], [-overallheight/2, overallheight/2], 'g-', 'Linewidth', 5);
plot([overallbase/2, -overallbase/2], [overallheight/2, overallheight/2], 'g-', 'Linewidth', 5);
plot([-overallbase/2, -overallbase/2], [overallheight/2, -overallheight/2], 'g-', 'Linewidth', 5);

plot([-lowerbase/2,lowerbase/2],[lowercenter-lowerheight/2,lowercenter-lowerheight/2], 'b-', 'Linewidth', 5);
plot([lowerbase/2,lowerbase/2],[lowercenter-lowerheight/2,lowercenter+lowerheight/2], 'b-', 'Linewidth', 5);
plot([lowerbase/2,-lowerbase/2],[lowercenter+lowerheight/2,lowercenter+lowerheight/2], 'b-', 'Linewidth', 5);
plot([-lowerbase/2,-lowerbase/2],[lowercenter+lowerheight/2,lowercenter-lowerheight/2], 'b-', 'Linewidth', 5);

plot([-upperbase/2,upperbase/2],[uppercenter-upperheight/2,uppercenter-upperheight/2], 'b-', 'Linewidth', 5);
plot([upperbase/2,upperbase/2],[uppercenter-upperheight/2,uppercenter+upperheight/2], 'b-', 'Linewidth', 5);
plot([upperbase/2,-upperbase/2],[uppercenter+upperheight/2,uppercenter+upperheight/2], 'b-', 'Linewidth', 5);
plot([-upperbase/2,-upperbase/2],[uppercenter+upperheight/2,uppercenter-upperheight/2], 'b-', 'Linewidth', 5);

unitsize = size/20;

norms = sensorpositions(:,3:4).*unitsize;
quiver(sensorpositions(:,1),sensorpositions(:,2),norms(:,1),norms(:,2));

viscircles(objects(:,1:2),objects(:,3));