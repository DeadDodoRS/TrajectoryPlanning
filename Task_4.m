clear; clc;

% Set joints as symbolical
syms q1 real
syms q2 real
syms q3 real

L = [1 1 1]

qStart = [radtodeg(1) radtodeg(0) radtodeg(1)];
qFinish = [radtodeg(sqrt(2)/2) radtodeg(sqrt(2)/2) radtodeg(1.2)];

FK = simplify(Rz(q1)*Tz(L(1))*Ry(q2)*Tx(L(2))*Ry(q3)*Tx(L(3)))

% Find FK for start pos
startPos = subs(FK, [q1, q2, q3], [qStart(1), qStart(2), qStart(3)])

% Find FK for finish pos
finishPos = subs(FK, [q1, q2, q3], [qFinish(1), qFinish(2), qFinish(3)])

% IK


% Visualisation
figure;
hold on
view(200,25)
grid on
axis equal

% Start position
T1=Rz(qStart(1))*Tz(L(1));
plot3([0 T1(1,4)],[0 T1(2,4)],[0 T1(3,4)],'-r','LineWidth', 5);

T2=Rz(qStart(1))*Tz(L(1))*Ry(qStart(2))*Tx(L(2));
plot3([T1(1,4) T2(1,4)],[T1(2,4) T2(2,4)],[T1(3,4) T2(3,4)],'-r','LineWidth', 5);

st_T3=Rz(qStart(1))*Tz(L(1))*Ry(qStart(2))*Tx(L(2))*Ry(qStart(3))*Tx(L(3));
p1 = plot3([T2(1,4) st_T3(1,4)],[T2(2,4) st_T3(2,4)],[T2(3,4) st_T3(3,4)],'-r','LineWidth', 5);


% Finish position
T1=Rz(qFinish(1))*Tz(L(1));
plot3([0 T1(1,4)],[0 T1(2,4)],[0 T1(3,4)],'-g','LineWidth', 5);

T2=Rz(qFinish(1))*Tz(L(1))*Ry(qFinish(2))*Tx(L(2));
plot3([T1(1,4) T2(1,4)],[T1(2,4) T2(2,4)],[T1(3,4) T2(3,4)],'-g','LineWidth', 5);

fin_T3=Rz(qFinish(1))*Tz(L(1))*Ry(qFinish(2))*Tx(L(2))*Ry(qFinish(3))*Tx(L(3));
p2 = plot3([T2(1,4) fin_T3(1,4)],[T2(2,4) fin_T3(2,4)],[T2(3,4) fin_T3(3,4)],'-g','LineWidth', 5);

% Line between start and final
p3 = plot3([st_T3(1,4) fin_T3(1,4)],[st_T3(2,4) fin_T3(2,4)],[st_T3(3,4) fin_T3(3,4)],'Color', [50, 50, 50, 50] / 255,'LineWidth', 5);

legend([p1 p2 p3], [{'Start pos'},{'Finish pos'},{'Move trajectory'}])

