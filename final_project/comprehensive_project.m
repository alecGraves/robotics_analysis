%% arm params
d1 = 0.2755;
d2 = 0.2050;
d3 = 0.2050;
d4 = 0.2073;
d5 = 0.1038;
d6 = 0.1038;
d7 = 0.1600;
e2 = 0.0098;

%% useful poses
qz = [0       8*pi/3  0       pi/3    pi/2    pi/2    pi      ]; % zero angles
qr = [180     180     180     180     180     180     180     ;
      0       100     280     65      10      210     210     ;
      273     183     390     49      258     288     288     ;
      323     210     166     88      190     233     233     ;
      283     163     0       44      265     258     258     ];
qr = deg2rad(qr);

%% set up symbols
pi_s = sym(pi);
% pi = sym(pi)
syms q1 q2 q3 q4 q5 q6 q7
joint_vars = [q1 q2 q3 q4 q5 q6 q7];
arrayfun(@(var) assume(var,'real'), joint_vars);

%% forward kinematics
hw_0 = to_homogeneous([1, 0, 0; 0, -1, 0; 0, 0, -1]);
h0_1 = dh_transform(0, -d1, pi_s/2, q1+pi_s);
h1_2 = dh_transform(0, 0, pi_s/2, q2);
h2_3 = dh_transform(0, -(d2+d3), pi_s/2, q3);
h3_4 = dh_transform(0, -e2, pi_s/2, q4);
h4_5 = dh_transform(0, -(d4+d5), pi_s/2, q5);
h5_6 = dh_transform(0, 0, pi_s/2, q6);
h6_7 = dh_transform(0, -(d6+d7), pi_s, q7+pi_s/2);

homogeneous_tfs = {hw_0, h0_1, h1_2,  h2_3, h3_4, h4_5, h5_6, h6_7};
cumulative_homogeneous_tfs = homogeneous_tfs;

for i = 2:length(homogeneous_tfs)
    cumulative_homogeneous_tfs{i} = cumulative_homogeneous_tfs{i-1}*cumulative_homogeneous_tfs{i};
end
h0_7 = cumulative_homogeneous_tfs{end};

% jacobian = get_jacobian(h0_7, joint_vars) % long and expensive op

numposes = size(qr);
numposes = numposes(1);
for i = 1:numposes
    qr(i, :)
    current = double(sym_replace(h0_7, joint_vars, qr(i, :)))*100; % numeric cm
    position = current(1:3, 4)
    fprintf('%f\t%f\t%f\t', position(1), position(2), position(3))
    pause()
end
disp('poses complete')
pause()

%MDL_JACO2_7DOF Create model of JACO2 7DOF arm
%
%      mdl_jaco2_7dof
%
% Script creates the workspace variable JACO2 which describes the
% kinematic and dynamic characteristics of the Kinova Jaco2 arm.
%
% Also defines the vectors:
%   qz   zero joint angle configuration.

%% define DH table
startup_rtb
clear L

%      th    d       a    alpha
DH = [0     d1     0    pi/2      ;
      0     0      0    pi/2      ;
      0     d2+d3  0    pi/2      ;
      0     e2     0    pi/2      ;
      0     d4+d5  0    pi/2      ;
      0     0      0    pi/2      ;
      0     d6+d7  0    pi        ];

size_DH = size(DH);

%% define robotic arm
for i = 1:size_DH(1)
   L(i) = Link(DH(i,1:end));
end

jaco2 = SerialLink(L, 'name', 'JACO2', 'manufacturer', 'Kinova');



%% initiate plots
fig_1 = figure(1);
set(fig_1, 'Name', 'Isometric View', 'Units', 'normalized', ...
   'Position', [0.5,0.02,0.5,0.39]);
fig_2 = figure(2);
set(fig_2, 'Name', 'Top View', 'Units', 'normalized', ...
           'Position', [0,0.02,0.5,0.39]);
fig_3 = figure(3);
set(fig_3, 'Name', 'Left View', 'Units', 'normalized', ...
           'Position', [0,0.52,0.5,0.39]);
fig_4 = figure(4);
set(fig_4, 'Name', 'Right View', 'Units', 'normalized', ...
           'Position', [0.5,0.52,0.5,0.39]);
%
% The trajectory demonstration has shown how a joint coordinate trajectory
% may be generated
t = [0:.056:2]';     % generate a time vector
qr_size = size(qr);

%% iterate through poses
for i = 1:qr_size(1)
   q = jtraj(qz, qr(i,1:end), t);

   figure(1);
   jaco2.plot(qz);
   view(60,40)

   figure(2);
   jaco2.plot(qz);
   view(180,90)

   figure(3);
   jaco2.plot(qz);
   view(-180,0)

   figure(4);
   jaco2.plot(qz);
   view(90,0)

   jaco2.plot(q)

   qz = qr(i,1:end);
end

jaco2.teach()
