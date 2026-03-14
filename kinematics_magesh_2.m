% Initialization

clc

clear

close all

% camera data Inputs

x = input("enter the x coordinate : ");

y = input("enter the y coordinate : ");

z = input("enter the z coordinate : ");

% link lengths

l1 = 0.45;

l2 = 0.35;

l3 = 0.15;

% orientation of end effector

O = -90;

theta = zeros(4,1);

%% Inverse Kinematics

theta1 = atan2d(y,x);

X = sqrt(x^2 + y^2);

Y = z;

a = X - l3*cosd(O);

b = Y - l3*sind(O);

r = sqrt(a^2 + b^2);

% Compute elbow solutions

D = (l1^2 + l2^2 - r^2)/(2 * l1 * l2);

theta3_down = 180 - acosd(D);

theta3_up   = 180 + acosd(D);

theta2_down = atan2d(b,a) - acosd((l1^2 + r^2 - l2^2)/(2*l1*r));

theta2_up   = atan2d(b,a) + acosd((l1^2 + r^2 - l2^2)/(2*l1*r));

%% Test both configurations

configs = [theta2_down theta3_down;

theta2_up   theta3_up];

valid = false;

for i = 1:2

t1 = theta1;

t2 = configs(i,1);

t3 = configs(i,2);



% Joint positions



P0 = [0 0 0];



V1 = [l1*cosd(t2)*cosd(t1), ...

      l1*cosd(t2)*sind(t1), ...

      l1*sind(t2)];



P2 = P0 + V1;



V2 = [l2*cosd(t2+t3)*cosd(t1), ...

      l2*cosd(t2+t3)*sind(t1), ...

      l2*sind(t2+t3)];



P3 = P2 + V2;



V3 = [l3*cosd(O)*cosd(t1), ...

      l3*cosd(O)*sind(t1), ...

      l3*sind(O)];



P4 = P3 + V3;



% Check ground constraint



if P2(3) >= 0 && P3(3) >= 0 && P4(3) >= 0

    valid = true;

    theta2 = t2;

    theta3 = t3;

    break

end

end

if ~valid

error("No valid configuration exists with all joints above ground.")

end

theta4 = O - (theta2 + theta3);

theta = [theta1 theta2 theta3 theta4];

%% Final Joint Positions

t1 = theta1;

t2 = theta2;

t3 = theta3;

P0 = [0 0 0];

V1 = [l1*cosd(t2)*cosd(t1),l1*cosd(t2)*sind(t1),  l1*sind(t2)];

P2 = P0 + V1;

V2 = [l2*cosd(t2+t3)*cosd(t1), l2*cosd(t2+t3)*sind(t1), l2*sind(t2+t3)];

P3 = P2 + V2;

V3 = [l3*cosd(O)*cosd(t1), l3*cosd(O)*sind(t1), l3*sind(O)];

P4 = P3 + V3;

%% Plot Robot

figure

hold on

grid on

axis equal

plot3([P0(1) P2(1)],[P0(2) P2(2)],[P0(3) P2(3)],'LineWidth',3)

plot3([P2(1) P3(1)],[P2(2) P3(2)],[P2(3) P3(3)],'LineWidth',3)

plot3([P3(1) P4(1)],[P3(2) P4(2)],[P3(3) P4(3)],'LineWidth',3)

scatter3(P0(1),P0(2),P0(3),80,'filled')

scatter3(P2(1),P2(2),P2(3),80,'filled')

scatter3(P3(1),P3(2),P3(3),80,'filled')

scatter3(P4(1),P4(2),P4(3),80,'filled')

xlabel('X')

ylabel('Y')

zlabel('Z')

title('4 DOF Robotic Arm (All Joints z ≥ 0)')

view(45,30)

%% Convert angles to motor-friendly range

% wrap to -180 to 180

for i = 1:length(theta)

if theta(i)>0 

    theta(i) = theta(i) - 90;

end

if theta(i)<0

    theta(i) = theta(i) + 90;

end

end

disp('Motor Input Angles [theta1 theta2 theta3 theta4] = ')

disp(theta)