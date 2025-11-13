%% BE3090 A3 – 2 link arm drawing the letter P
% okay, so this whole script is my attempt to go from the theory
% of a 2 link arm to actually drawing the letter P in task space.
clear; clc; close all;

%% link lengths (given)
% first thing is just to store the link lengths that the assignment gave us.
% everything else in the code depends on these two numbers.
r1 = 47;     % first link length in mm
r2 = 63;     % second link length in mm

%% (a) FORWARD KINEMATICS
% here i write the forward kinematics in compact form.
% if i know theta1 and theta2, this function tells me where the tip P is.
% x = r1*cos(theta1) + r2*cos(theta1 + theta2)
% y = r1*sin(theta1) + r2*sin(theta1 + theta2)
fk = @(t1,t2) [ ...
    r1*cos(t1) + r2*cos(t1 + t2); ...
    r1*sin(t1) + r2*sin(t1 + t2)];

%% (b) WORKSPACE: all possible P for 0 < theta1 < pi/2, 0 < theta2 < pi
% now i want to see what the arm can reach if i sweep through all valid angles.
% so here i build a grid of theta1 and theta2 values and map them to x,y.
theta1 = linspace(0, pi/2, 100);     % joint 1 grid from 0 to 90 degrees
theta2 = linspace(0, pi,   100);     % joint 2 grid from 0 to 180 degrees
[Th1, Th2] = meshgrid(theta1, theta2);  % all combinations of angles

% apply forward kinematics on the whole grid
X = r1*cos(Th1) + r2*cos(Th1 + Th2);
Y = r1*sin(Th1) + r2*sin(Th1 + Th2);

% plot the reachable workspace as a scatter plot
figure;
scatter(X(:), Y(:), 5, 'filled');
axis equal;
xlabel('x (mm)');
ylabel('y (mm)');
title('Reachable workspace of 2 link arm');

%% (c) IK FOR A SPECIFIC POINT: P = (20, 72) mm
% here i test the inverse kinematics for one specific target, (20,72).
% the idea is: if the point is reachable, what joint angles would i need.
xT = 20;
yT = 72;

[okPT, th1_solutions, th2_solutions] = twoLinkIK(xT, yT, r1, r2);

disp('--- Part (c): IK solutions for P = (20,72) mm ---');
if okPT
    % there are up to two possible solutions: elbow up and elbow down.
    % i convert from radians to degrees so it is easier to read.
    th1_deg = th1_solutions * 180/pi;
    th2_deg = th2_solutions * 180/pi;
    disp(table(th1_deg.', th2_deg.', ...
        'VariableNames', {'theta1_deg','theta2_deg'}));
else
    % if we ever end up here it means the point lies outside the workspace.
    disp('Point (20,72) is not reachable with given link lengths.');
end

%% (d) FOUR KEY POINTS FOR LETTER P
% now i choose four landmark points that capture the main shape of the P.
% i picked coordinates that are reachable and roughly match the sketch.
%   P1: bottom of stem
%   P2: mid stem
%   P3: top of stem
%   P4: right side of the bowl
P1 = [40; 20];   % bottom stem
P2 = [40; 50];   % middle stem
P3 = [40; 90];   % top stem
P4 = [60; 80];   % right side of bowl (moved a bit closer to stay reachable)

P_all = [P1 P2 P3 P4];

% here i preallocate arrays for the joint angles at those four feature points.
theta1_pts = zeros(1,4);
theta2_pts = zeros(1,4);

% loop through the four points and solve the IK for each one
for k = 1:4
    [ok, t1s, t2s] = twoLinkIK(P_all(1,k), P_all(2,k), r1, r2);
    if ~ok
        % if a point is not reachable i want to know right away
        error('Chosen point %d is unreachable, adjust coordinates.', k);
    end
    % i pick the first solution returned, which corresponds to one elbow
    % configuration (here i treat it as the elbow up branch).
    theta1_pts(k) = t1s(1);
    theta2_pts(k) = t2s(1);
end

% print the table of feature points and their corresponding joint angles
disp('--- Part (d): angles for 4 feature points ---');
disp(table((1:4).', P_all(1,:).', P_all(2,:).', ...
    theta1_pts.'*180/pi, theta2_pts.'*180/pi, ...
    'VariableNames',{'point','x_mm','y_mm','theta1_deg','theta2_deg'}));

%% (e) ANIMATE DRAWING THE LETTER P WITHOUT LIFTING THE PEN
% okay, now i move from just four points to a whole path.
% the idea is to sample a bunch of points along the outline of P,
% find the joint angles for each one, and then animate the arm moving
% so that the tip traces the letter.

% first i define the geometry of the letter in task space:
baseX = 40;           % x coordinate of the stem
baseY = 20;           % y coordinate of bottom of the stem
stemHeight = 75;      % total height of the vertical stem
topWidth   = 40;      % how far the bowl sticks out to the right
bowlRadius = 20;      % radius for the curved part of the bowl

% 1) vertical stem (bottom to top)
% here i keep x fixed at baseX and move y from baseY to baseY + stemHeight.
x1 = baseX*ones(1,25);
y1 = linspace(baseY, baseY + stemHeight, 25);

% 2) top horizontal (left to right)
% now i move across the top of the P.
x2 = linspace(baseX, baseX + topWidth, 20);
y2 = (baseY + stemHeight)*ones(1,20);

% 3) curved bowl (top right down toward mid level)
% i parameterize an arc using cos and sin. thetaArc controls how much of
% the circle i take. cx, cy is the center of that circle.
thetaArc = linspace(pi/2, -pi/2, 25);
cx = baseX + topWidth;              % center x of the arc
cy = baseY + stemHeight - bowlRadius; % center y of the arc
x3 = cx + bowlRadius*cos(thetaArc);
y3 = cy + bowlRadius*sin(thetaArc);

% 4) line back to stem at mid height
% finally, i connect the end of the arc back to the stem at half height.
meetY = baseY + stemHeight/2;
x4 = linspace(x3(end), baseX, 15);
y4 = linspace(y3(end), meetY, 30);

% combine all four segments into one long list of points
xPath = [x1 x2 x3 x4];
yPath = [y1 y2 y3 y4];
nPts  = numel(xPath);

% here i preallocate the arrays where i will store the joint angles
theta1_path = nan(1,nPts);
theta2_path = nan(1,nPts);

% set up the figure for the animation
figure;
hold on;
axis equal;
xlabel('x (mm)');
ylabel('y (mm)');
title('drawing the letter P');

% main animation loop: go through each path point, do IK, then draw
for k = 1:nPts
    % solve inverse kinematics for the current path point
    [ok, t1s, t2s] = twoLinkIK(xPath(k), yPath(k), r1, r2);
    if ~ok
        % if any point is slightly outside the workspace i just warn and skip it
        warning('Path point %d is unreachable', k);
        continue;
    end

    % again i pick the first solution (consistent elbow configuration).
    theta1 = t1s(1);
    theta2 = t2s(1);

    theta1_path(k) = theta1;
    theta2_path(k) = theta2;

    % now i use forward kinematics to find the positions of the joints
    p0 = [0; 0];              % base joint at the origin
    p1 = fk(theta1, 0);       % end of the first link
    p2 = fk(theta1, theta2);  % location of the tip P

    % plot the trace of the letter so far in red
    plot(xPath(1:k), yPath(1:k), 'r.', 'MarkerSize', 10);
    % and plot the current configuration of the arm in blue
   if k == 1
    armPlot = plot([p0(1) p1(1) p2(1)], [p0(2) p1(2) p2(2)], ...
        'b-o','LineWidth',2,'MarkerFaceColor','b');
else
    set(armPlot, 'XData',[p0(1) p1(1) p2(1)], ...
                 'YData',[p0(2) p1(2) p2(2)]);
    %this is to make sure that the vectors at each point dont overlap and
    %there is a clean (blue arm) vector.
end
    xlim([-50 200]);
    ylim([-50 160]);
    drawnow;   % update the plot so it looks animated
end

disp('Animation complete.');

%% ================== INVERSE KINEMATICS FUNCTION =========================
function [ok, t1_solutions, t2_solutions] = twoLinkIK(x, y, r1, r2)
% twoLinkIK:
% okay, so this helper function does the actual math for inverse kinematics.
% given a target point (x,y) and link lengths r1 and r2,
% it figures out the two possible pairs of angles (theta1, theta2)
% that place the tip at that point, if the point is reachable.

    % squared distance from the origin to the point
    d2 = x^2 + y^2;

    % use the law of cosines to get cos(theta2)
    c2 = (d2 - r1^2 - r2^2) / (2*r1*r2);  % cos(theta2)

    % if |c2| > 1 that means the geometry does not work
    if abs(c2) > 1
        ok = false;
        t1_solutions = [];
        t2_solutions = [];
        return;
    end

    % otherwise, compute sin(theta2). there are two possible signs.
    s2 = sqrt(1 - c2^2);

    % elbow up solution
    t2a = atan2(s2, c2);
    k1a = r1 + r2*cos(t2a);
    k2a = r2*sin(t2a);
    t1a = atan2(y, x) - atan2(k2a, k1a);

    % elbow down solution
    t2b = atan2(-s2, c2);
    k1b = r1 + r2*cos(t2b);
    k2b = r2*sin(t2b);
    t1b = atan2(y, x) - atan2(k2b, k1b);

    % collect both solutions
    t1_solutions = [t1a, t1b];
    t2_solutions = [t2a, t2b];
    ok = true;
end