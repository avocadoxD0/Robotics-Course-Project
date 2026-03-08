%% Initialization
clc
clear
close all

%% Define symbolic variables
syms theta1 theta2 theta3 theta4 real
syms l1 l2 l3 d2 d3 d4 real

%% Modified DH Parameter Table
% columns: [alpha_i-1  a_i-1  d_i  theta_i]

DH = [ 0      0    0   theta1;
       pi/2   l1   0  theta2;
       0      l2   0  theta3;
       -1*pi/2 l3   0  theta4 ];

%% Define Modified DH Transformation Function

mdh = @(alpha,a,d,theta) ...
[ cos(theta) -sin(theta) 0 a;
  cos(alpha)*sin(theta) cos(alpha)*cos(theta) -sin(alpha) -sin(alpha)*d;
  sin(alpha)*sin(theta) sin(alpha)*cos(theta) cos(alpha) cos(alpha)*d;
  0 0 0 1 ];

%% Compute Individual Transformations

T01 = mdh(DH(1,1),DH(1,2),DH(1,3),DH(1,4));
T12 = mdh(DH(2,1),DH(2,2),DH(2,3),DH(2,4));
T23 = mdh(DH(3,1),DH(3,2),DH(3,3),DH(3,4));
T34 = mdh(DH(4,1),DH(4,2),DH(4,3),DH(4,4));

%% Display Individual Matrices

disp('T01 = ')
disp(T01)

disp('T12 = ')
disp(T12)

disp('T23 = ')
disp(T23)

disp('T34 = ')
disp(T34)
%% Compute Full Forward Kinematics

T04 = simplify(T01*T12*T23*T34);

disp('T04 = ')
disp(T04)
%% Extract End Effector Position

Px = simplify(T04(1,4));
Py = simplify(T04(2,4));
Pz = simplify(T04(3,4));

disp('End Effector Position:')
disp([Px;Py;Pz])

%%
%% Substitute actual robot parameters

L1 = 0.45;
L2 = 0.35;
L3 = 0.15;

Px_num = subs(Px,{l1,l2,l3},{L1,L2,L3});
Py_num = subs(Py,{l1,l2,l3},{L1,L2,L3});
Pz_num = subs(Pz,{l1,l2,l3},{L1,L2,L3});

disp('Px with robot parameters = ')
disp(Px_num)

disp('Py with robot parameters = ')
disp(Py_num)

disp('Pz with robot parameters = ')
disp(Pz_num)


%% Evaluate FK for a sample configuration

theta1_val = deg2rad(10);
theta2_val = deg2rad(20);
theta3_val = deg2rad(30);
theta4_val = deg2rad(40);

Px_val = double(subs(Px_num,{theta1,theta2,theta3,theta4},...
    {theta1_val,theta2_val,theta3_val,theta4_val}));

Py_val = double(subs(Py_num,{theta1,theta2,theta3,theta4},...
    {theta1_val,theta2_val,theta3_val,theta4_val}));

Pz_val = double(subs(Pz_num,{theta1,theta2,theta3,theta4},...
    {theta1_val,theta2_val,theta3_val,theta4_val}));

disp('Forward Kinematics Position:')
disp([Px_val Py_val Pz_val])
%%
%% Inverse Kinematics

% Desired end effector position
px = 0.8620;
py = 0.1520;
pz = 0.2346;

% Link lengths
l1 = 0.45;
l2 = 0.35;
l3 = 0.15;

%% Solve theta1

theta1 = atan2(py,px);

disp('theta1 = ')
disp(rad2deg(theta1))

%% Compute radial distance

r = sqrt(px^2 + py^2);

r_prime = r - l1;

%% Compute theta3

D = (r_prime^2 + pz^2 - l2^2 - l3^2)/(2*l2*l3);

% Clamp D to avoid numerical errors
D = max(min(D,1),-1);

theta3 = atan2(sqrt(1-D^2),D);   % elbow-down

disp('theta3 = ')
disp(rad2deg(theta3))


%% Compute theta2

phi = atan2(pz,r_prime);

psi = atan2(l3*sin(theta3), l2 + l3*cos(theta3));

theta2 = phi - psi;

disp('theta2 = ')
disp(rad2deg(theta2))

%% Compute theta4 (choose desired orientation)

theta4 = 0;

disp('theta4 = ')
disp(rad2deg(theta4))

%% Display final joint vector

disp('Joint Angles [theta1 theta2 theta3 theta4] = ')
disp(rad2deg([theta1 theta2 theta3 theta4]))
