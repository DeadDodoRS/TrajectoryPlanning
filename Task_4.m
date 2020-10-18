clear; clc;

% Set joints as symbolical
syms q1 real
syms q2 real
syms q3 real

L = [1 1 1];

qStart = [1 0 1];
qFinish = [sqrt(2)/2 sqrt(2)/2 1.2];

FK = simplify(Rz(q1)*Tz(L(1))*Ry(q2)*Tx(L(2))*Ry(q3)*Tx(L(3)));

% Find FK for start pos
startPos = vpa(simplify(subs(FK, [q1, q2, q3], [wrapToPi(qStart(1)), wrapToPi(qStart(2)), wrapToPi(qStart(3))])))

% Find FK for finish pos
finishPos = vpa(simplify(subs(FK, [q1, q2, q3], [wrapToPi(qFinish(1)), wrapToPi(qFinish(2)), wrapToPi(qFinish(3))])));

% IK
X = startPos(1, 4);
Y = startPos(2, 4);
Z = startPos(3, 4);

%test = qStart(1)
test = wrapToPi(qStart(1))
q1 = atan2(Y, X)

test = wrapToPi(qStart(3))
AB = sqrt((X^2 + Y^2) + (Z - L(1))^2);
q3 = vpa(pi - acos((L(3)^2 + L(2)^2 - AB^2) / (2 * L(2) * L(3))))

test = wrapToPi(qStart(2))
q21 = atan2(Z - L(1), (X^2 + Y^2));
q22 = acos((L(2)^2 + AB^2 - L(3)^2)/(2 * L(2) * AB));

q2 = q21 + q22

% Visualisation
figure;
hold on
view(200,25)
grid on
axis equal

% Start position
T1=Rz(rad2deg(qStart(1)))*Tz(L(1));
plot3([0 T1(1,4)],[0 T1(2,4)],[0 T1(3,4)],'-r','LineWidth', 5);

T2=Rz(rad2deg(qStart(1)))*Tz(L(1))*Ry(rad2deg(qStart(2)))*Tx(L(2));
plot3([T1(1,4) T2(1,4)],[T1(2,4) T2(2,4)],[T1(3,4) T2(3,4)],'-r','LineWidth', 5);

st_T3=Rz(rad2deg(qStart(1)))*Tz(L(1))*Ry(rad2deg(qStart(2)))*Tx(L(2))*Ry(rad2deg(qStart(3)))*Tx(L(3));
p1 = plot3([T2(1,4) st_T3(1,4)],[T2(2,4) st_T3(2,4)],[T2(3,4) st_T3(3,4)],'-r','LineWidth', 5);


% Finish position
T1=Rz(rad2deg(qFinish(1)))*Tz(L(1));
plot3([0 T1(1,4)],[0 T1(2,4)],[0 T1(3,4)],'-g','LineWidth', 5);

T2=Rz(rad2deg(qFinish(1)))*Tz(L(1))*Ry(rad2deg(qFinish(2)))*Tx(L(2));
plot3([T1(1,4) T2(1,4)],[T1(2,4) T2(2,4)],[T1(3,4) T2(3,4)],'-g','LineWidth', 5);

fin_T3=Rz(rad2deg(qFinish(1)))*Tz(L(1))*Ry(rad2deg(qFinish(2)))*Tx(L(2))*Ry(rad2deg(qFinish(3)))*Tx(L(3));
p2 = plot3([T2(1,4) fin_T3(1,4)],[T2(2,4) fin_T3(2,4)],[T2(3,4) fin_T3(3,4)],'-g','LineWidth', 5);

% Line between start and final
p3 = plot3([st_T3(1,4) fin_T3(1,4)],[st_T3(2,4) fin_T3(2,4)],[st_T3(3,4) fin_T3(3,4)],'Color', [50, 50, 50, 50] / 255,'LineWidth', 5);

legend([p1 p2 p3], [{'Start pos'},{'Finish pos'},{'Move trajectory'}])


