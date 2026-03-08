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

disp('T01 = ') %[output:7bfdfcf3]
disp(T01) %[output:0d06b59f]

disp('T12 = ') %[output:96ebd192]
disp(T12) %[output:12145ff4]

disp('T23 = ') %[output:78ddd539]
disp(T23) %[output:5f9d2197]

disp('T34 = ') %[output:5f612f5f]
disp(T34) %[output:3f956429]

%% Compute Full Forward Kinematics

T04 = simplify(T01*T12*T23*T34);

disp('T04 = ') %[output:6fc65f85]
disp(T04) %[output:388d0897]

%% Extract End Effector Position

Px = simplify(T04(1,4));
Py = simplify(T04(2,4));
Pz = simplify(T04(3,4));

disp('End Effector Position:') %[output:5c39e6a1]
disp([Px;Py;Pz]) %[output:38fa64ee]

%%
%% Substitute actual robot parameters

L1 = 0.45;
L2 = 0.35;
L3 = 0.15;

Px_num = subs(Px,{l1,l2,l3},{L1,L2,L3});
Py_num = subs(Py,{l1,l2,l3},{L1,L2,L3});
Pz_num = subs(Pz,{l1,l2,l3},{L1,L2,L3});

disp('Px with robot parameters = ') %[output:16375aff]
disp(Px_num) %[output:4c38cb87]

disp('Py with robot parameters = ') %[output:44eeca67]
disp(Py_num) %[output:6564ed79]

disp('Pz with robot parameters = ') %[output:657bd52d]
disp(Pz_num) %[output:339ed2b7]


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

disp('Forward Kinematics Position:') %[output:92f50a5f]
disp([Px_val Py_val Pz_val]) %[output:1a293ee7]
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

disp('theta1 = ') %[output:96ee5679]
disp(rad2deg(theta1)) %[output:372be0b8]

%% Compute radial distance

r = sqrt(px^2 + py^2);

r_prime = r - l1;

%% Compute theta3

D = (r_prime^2 + pz^2 - l2^2 - l3^2)/(2*l2*l3);

% Clamp D to avoid numerical errors
D = max(min(D,1),-1);

theta3 = atan2(sqrt(1-D^2),D);   % elbow-down

disp('theta3 = ') %[output:888edca0]
disp(rad2deg(theta3)) %[output:09cbecda]


%% Compute theta2

phi = atan2(pz,r_prime);

psi = atan2(l3*sin(theta3), l2 + l3*cos(theta3));

theta2 = phi - psi;

disp('theta2 = ') %[output:23d06154]
disp(rad2deg(theta2)) %[output:1e66f0ef]

%% Compute theta4 (choose desired orientation)

theta4 = 0;

disp('theta4 = ') %[output:4cd6173b]
disp(rad2deg(theta4)) %[output:8612c86c]

%% Display final joint vector

disp('Joint Angles [theta1 theta2 theta3 theta4] = ') %[output:40c939ae]
disp(rad2deg([theta1 theta2 theta3 theta4])) %[output:72ad04bc]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":44}
%---
%[output:7bfdfcf3]
%   data: {"dataType":"text","outputData":{"text":"T01 = \n","truncated":false}}
%---
%[output:0d06b59f]
%   data: {"dataType":"symbolic","outputData":{"name":"","value":"\\left(\\begin{array}{cccc}\n\\cos \\left(\\theta_1 \\right) & -\\sin \\left(\\theta_1 \\right) & 0 & 0\\\\\n\\sin \\left(\\theta_1 \\right) & \\cos \\left(\\theta_1 \\right) & 0 & 0\\\\\n0 & 0 & 1 & 0\\\\\n0 & 0 & 0 & 1\n\\end{array}\\right)"}}
%---
%[output:96ebd192]
%   data: {"dataType":"text","outputData":{"text":"T12 = \n","truncated":false}}
%---
%[output:12145ff4]
%   data: {"dataType":"symbolic","outputData":{"name":"","value":"\\left(\\begin{array}{cccc}\n\\cos \\left(\\theta_2 \\right) & -\\sin \\left(\\theta_2 \\right) & 0 & l_1 \\\\\n0 & 0 & -1 & 0\\\\\n\\sin \\left(\\theta_2 \\right) & \\cos \\left(\\theta_2 \\right) & 0 & 0\\\\\n0 & 0 & 0 & 1\n\\end{array}\\right)"}}
%---
%[output:78ddd539]
%   data: {"dataType":"text","outputData":{"text":"T23 = \n","truncated":false}}
%---
%[output:5f9d2197]
%   data: {"dataType":"symbolic","outputData":{"name":"","value":"\\left(\\begin{array}{cccc}\n\\cos \\left(\\theta_3 \\right) & -\\sin \\left(\\theta_3 \\right) & 0 & l_2 \\\\\n\\sin \\left(\\theta_3 \\right) & \\cos \\left(\\theta_3 \\right) & 0 & 0\\\\\n0 & 0 & 1 & 0\\\\\n0 & 0 & 0 & 1\n\\end{array}\\right)"}}
%---
%[output:5f612f5f]
%   data: {"dataType":"text","outputData":{"text":"T34 = \n","truncated":false}}
%---
%[output:3f956429]
%   data: {"dataType":"symbolic","outputData":{"name":"","value":"\\left(\\begin{array}{cccc}\n\\cos \\left(\\theta_4 \\right) & -\\sin \\left(\\theta_4 \\right) & 0 & l_3 \\\\\n0 & 0 & 1 & 0\\\\\n-\\sin \\left(\\theta_4 \\right) & -\\cos \\left(\\theta_4 \\right) & 0 & 0\\\\\n0 & 0 & 0 & 1\n\\end{array}\\right)"}}
%---
%[output:6fc65f85]
%   data: {"dataType":"text","outputData":{"text":"T04 = \n","truncated":false}}
%---
%[output:388d0897]
%   data: {"dataType":"symbolic","outputData":{"name":"","value":"\\begin{array}{l}\n\\left(\\begin{array}{cccc}\n-\\sin \\left(\\theta_1 \\right)\\,\\sin \\left(\\theta_4 \\right)-\\cos \\left(\\theta_4 \\right)\\,\\sigma_2  & \\sin \\left(\\theta_4 \\right)\\,\\sigma_2 -\\cos \\left(\\theta_4 \\right)\\,\\sin \\left(\\theta_1 \\right) & -\\sin \\left(\\theta_2 +\\theta_3 \\right)\\,\\cos \\left(\\theta_1 \\right) & \\cos \\left(\\theta_1 \\right)\\,\\sigma_3 \\\\\n\\cos \\left(\\theta_1 \\right)\\,\\sin \\left(\\theta_4 \\right)-\\cos \\left(\\theta_4 \\right)\\,\\sigma_1  & \\cos \\left(\\theta_1 \\right)\\,\\cos \\left(\\theta_4 \\right)+\\sin \\left(\\theta_4 \\right)\\,\\sigma_1  & -\\sin \\left(\\theta_2 +\\theta_3 \\right)\\,\\sin \\left(\\theta_1 \\right) & \\sin \\left(\\theta_1 \\right)\\,\\sigma_3 \\\\\n\\sin \\left(\\theta_2 +\\theta_3 \\right)\\,\\cos \\left(\\theta_4 \\right) & -\\sin \\left(\\theta_2 +\\theta_3 \\right)\\,\\sin \\left(\\theta_4 \\right) & \\cos \\left(\\theta_2 +\\theta_3 \\right) & l_3 \\,\\sin \\left(\\theta_2 +\\theta_3 \\right)+l_2 \\,\\sin \\left(\\theta_2 \\right)\\\\\n0 & 0 & 0 & 1\n\\end{array}\\right)\\\\\n\\mathrm{}\\\\\n\\textrm{where}\\\\\n\\mathrm{}\\\\\n\\;\\;\\sigma_1 =\\sin \\left(\\theta_1 \\right)\\,\\sin \\left(\\theta_2 \\right)\\,\\sin \\left(\\theta_3 \\right)-\\cos \\left(\\theta_2 \\right)\\,\\cos \\left(\\theta_3 \\right)\\,\\sin \\left(\\theta_1 \\right)\\\\\n\\mathrm{}\\\\\n\\;\\;\\sigma_2 =\\cos \\left(\\theta_1 \\right)\\,\\sin \\left(\\theta_2 \\right)\\,\\sin \\left(\\theta_3 \\right)-\\cos \\left(\\theta_1 \\right)\\,\\cos \\left(\\theta_2 \\right)\\,\\cos \\left(\\theta_3 \\right)\\\\\n\\mathrm{}\\\\\n\\;\\;\\sigma_3 =l_1 +l_3 \\,\\cos \\left(\\theta_2 +\\theta_3 \\right)+l_2 \\,\\cos \\left(\\theta_2 \\right)\n\\end{array}"}}
%---
%[output:5c39e6a1]
%   data: {"dataType":"text","outputData":{"text":"End Effector Position:\n","truncated":false}}
%---
%[output:38fa64ee]
%   data: {"dataType":"symbolic","outputData":{"name":"","value":"\\left(\\begin{array}{c}\n\\cos \\left(\\theta_1 \\right)\\,{\\left(l_1 +l_3 \\,\\cos \\left(\\theta_2 +\\theta_3 \\right)+l_2 \\,\\cos \\left(\\theta_2 \\right)\\right)}\\\\\n\\sin \\left(\\theta_1 \\right)\\,{\\left(l_1 +l_3 \\,\\cos \\left(\\theta_2 +\\theta_3 \\right)+l_2 \\,\\cos \\left(\\theta_2 \\right)\\right)}\\\\\nl_3 \\,\\sin \\left(\\theta_2 +\\theta_3 \\right)+l_2 \\,\\sin \\left(\\theta_2 \\right)\n\\end{array}\\right)"}}
%---
%[output:16375aff]
%   data: {"dataType":"text","outputData":{"text":"Px with robot parameters = \n","truncated":false}}
%---
%[output:4c38cb87]
%   data: {"dataType":"symbolic","outputData":{"name":"","value":"\\cos \\left(\\theta_1 \\right)\\,{\\left(\\frac{3\\,\\cos \\left(\\theta_2 +\\theta_3 \\right)}{20}+\\frac{7\\,\\cos \\left(\\theta_2 \\right)}{20}+\\frac{9}{20}\\right)}"}}
%---
%[output:44eeca67]
%   data: {"dataType":"text","outputData":{"text":"Py with robot parameters = \n","truncated":false}}
%---
%[output:6564ed79]
%   data: {"dataType":"symbolic","outputData":{"name":"","value":"\\sin \\left(\\theta_1 \\right)\\,{\\left(\\frac{3\\,\\cos \\left(\\theta_2 +\\theta_3 \\right)}{20}+\\frac{7\\,\\cos \\left(\\theta_2 \\right)}{20}+\\frac{9}{20}\\right)}"}}
%---
%[output:657bd52d]
%   data: {"dataType":"text","outputData":{"text":"Pz with robot parameters = \n","truncated":false}}
%---
%[output:339ed2b7]
%   data: {"dataType":"symbolic","outputData":{"name":"","value":"\\frac{3\\,\\sin \\left(\\theta_2 +\\theta_3 \\right)}{20}+\\frac{7\\,\\sin \\left(\\theta_2 \\right)}{20}"}}
%---
%[output:92f50a5f]
%   data: {"dataType":"text","outputData":{"text":"Forward Kinematics Position:\n","truncated":false}}
%---
%[output:1a293ee7]
%   data: {"dataType":"text","outputData":{"text":"    0.8620    0.1520    0.2346\n\n","truncated":false}}
%---
%[output:96ee5679]
%   data: {"dataType":"text","outputData":{"text":"theta1 = \n","truncated":false}}
%---
%[output:372be0b8]
%   data: {"dataType":"text","outputData":{"text":"   10.0004\n\n","truncated":false}}
%---
%[output:888edca0]
%   data: {"dataType":"text","outputData":{"text":"theta3 = \n","truncated":false}}
%---
%[output:09cbecda]
%   data: {"dataType":"text","outputData":{"text":"   30.0179\n\n","truncated":false}}
%---
%[output:23d06154]
%   data: {"dataType":"text","outputData":{"text":"theta2 = \n","truncated":false}}
%---
%[output:1e66f0ef]
%   data: {"dataType":"text","outputData":{"text":"   19.9941\n\n","truncated":false}}
%---
%[output:4cd6173b]
%   data: {"dataType":"text","outputData":{"text":"theta4 = \n","truncated":false}}
%---
%[output:8612c86c]
%   data: {"dataType":"text","outputData":{"text":"     0\n\n","truncated":false}}
%---
%[output:40c939ae]
%   data: {"dataType":"text","outputData":{"text":"Joint Angles [theta1 theta2 theta3 theta4] = \n","truncated":false}}
%---
%[output:72ad04bc]
%   data: {"dataType":"text","outputData":{"text":"   10.0004   19.9941   30.0179         0\n\n","truncated":false}}
%---
