clc; clear;

% Set joints as symbolical
syms q1 real
syms q2 real
syms q3 real

% Set joints as values
rq1 = 0;
rq2 = 0;
rq3 = 0;

L1 = 1;
L2 = 1;
L3 = 1;

FK = simplify(Rz(q1)*Tz(L1)*Ry(q2)*Tx(L2)*Ry(q3)*Tx(q3));

% Task 1. Jacobian
R = simplify(FK(1:3,1:3));

% diff by q1
q1_diff=simplify(Rzd(q1)*Tz(L1)*Ry(q2)*Tx(L2)*Ry(q3)*Tx(L3)* [R^-1 zeros(3,1);0 0 0 1]);
J1= [q1_diff(1,4), q1_diff(2,4), q1_diff(3,4), q1_diff(3,2), q1_diff(1,3), q1_diff(2,1)]';

% diff by q2
q2_diff = simplify(Rz(q1)*Tz(L1)*Ryd(q2)*Tx(L2)*Ry(q3)*Tx(L3)* [R^-1 zeros(3,1);0 0 0 1]);
J2= [q2_diff(1,4), q2_diff(2,4), q2_diff(3,4), q2_diff(3,2), q2_diff(1,3), q2_diff(2,1)]';

% diff by q3
q3_diff = simplify(Rz(q1)*Tz(L1)*Ry(q2)*Tx(L2)*Ryd(q3)*Tx(L3)* [R^-1 zeros(3,1);0 0 0 1]);
J3= [q3_diff(1,4), q3_diff(2,4), q3_diff(3,4), q3_diff(3,2), q3_diff(1,3), q3_diff(2,1)]';

J_numerical= [simplify(J1), simplify(J2), simplify(J3)]

% Task 2
qStart = [0 0 0];
qFinish = [2 3 4];
tn = [0 2]

vStart = [0 0 0];
vFinish = [0 0 0];

accStart = [0 0 0];
accFinish = [0 0 0];

A = [1 tn(1) tn(1)^2 tn(1)^3 tn(1)^4 tn(1)^5
     0 1 2*tn(1) 3*tn(1)^2 4*tn(1)^3 5*tn(1)^4
     0 0 2 6*tn(1) 12*tn(1)^2 20*tn(1)^3
     
     1 tn(2) tn(2)^2 tn(2)^3 tn(2)^4 tn(2)^5
     0 1 2*tn(2) 3*tn(2)^2 4*tn(2)^3 5*tn(2)^4
     0 0 2 6*tn(2) 12*tn(2)^2 20*tn(2)^3]

c = [qStart; vStart; accStart; qFinish; vStart; accFinish]
b = A\c

%b(2,:)

t=linspace(tn(1),tn(end),100);

pos = zeros(3,100);
vel = zeros(3,100);
accel = zeros(3,100);

for i = 1:3
   for j = 1:100
       pos(i, j) = b(1, i) + b(2, i)*t(j) + b(3, i)*t(j)^2 + b(4, i)*t(j)^3 + b(5, i)*t(j)^4 + b(6, i)*t(j)^5;
       vel(i, j) = b(2, i) + 2*b(3, i)*t(j) + 3*b(4, i)*t(j)^2 + 4*b(5, i)*t(j)^3 + 5*b(6, i)*t(j)^4;
       accel(i, j) = 2*b(3, i) + 6*b(4, i)*t(j) + 12*b(5, i)*t(j)^2 + 20*b(6, i)*t(j)^3;
   end
end

figure
plot(t, pos)

figure
plot(t, vel)

figure
plot(t, accel)
 


% Task 3
qStart = [0 0 0];
qFinish = [2 3 4];

vStart = [0 0 0];
vFinish = [0 0 0];

accStart = [0 0 0];
accFinish = [0 0 0];

vMax = [1 1 1];
accMax = [10 10 10];

delta_t = 0.01;

% qStart = [10 0]
% qFinish = [25 40]
% 
% vStart = [0 0];
% vFinish = [0 0];
% 
% accStart = [0 0];
% accFinish = [0 0];
% 
% vMax = [2 5];
% accMax = [4.5 6];
% 
% delta_t = 0.01;

% Find time with taking into account delta t
n = 0;
while (floor(delta_t*10^n)~=delta_t*10^n)
    n=n+1;
end
E = 1*10^-n;

accelTimes = zeros(3,1);
accelTimes_new = zeros(3,1);

finalTimes = zeros(3,1);
finalTimes_new = zeros(3,1);

accelTimes(1) = vMax(1)/accMax(1);
if rem(accelTimes(1), delta_t)~=0
    accelTimes_new(1) = round(accelTimes(1),n)+E;
else
    accelTimes_new(1) = round(accelTimes(1),n);
end

finalTimes(1) = (qFinish(1)-qStart(1))/vMax(1) + accelTimes_new(1);
if rem(finalTimes(1), delta_t)~=0
    finalTimes_new(1) = round(finalTimes(1),n)+E;
else
    finalTimes_new(1) = round(finalTimes(1),n);
end

%TODO: Make it function. Too bad
accelTimes(2) = vMax(2)/accMax(2);
if rem(accelTimes(2), delta_t)~=0
    accelTimes_new(2) = round(accelTimes(2),n)+E;
else
    accelTimes_new(2) = round(accelTimes(2),n);
end

finalTimes(2) = (qFinish(2)-qStart(2))/vMax(2) + accelTimes_new(2);
if rem(finalTimes(2), delta_t)~=0
    finalTimes_new(2) = round(finalTimes(2),n)+E;
else
    finalTimes_new(2) = round(finalTimes(2),n);
end

accelTimes(3) = vMax(3)/accMax(3);
if rem(accelTimes(3), delta_t)~=0
    accelTimes_new(3) = round(accelTimes(3),n)+E;
else
    accelTimes_new(3) = round(accelTimes(3),n);
end

finalTimes(3) = (qFinish(3)-qStart(3))/vMax(3) + accelTimes_new(3);
if rem(finalTimes(3), delta_t)~=0
    finalTimes_new(3) = round(finalTimes(3),n)+E;
else
    finalTimes_new(3) = round(finalTimes(3),n);
end

tf_new = 0;
ta_new = 0;

max_final_time = -999;
for i = 1:3
    if finalTimes_new(i) > max_final_time
        max_final_time = finalTimes_new(i);
        
        tf_new = finalTimes_new(i);
        ta_new = accelTimes_new(i);
    end
end

vMax_new = zeros(3, 1);
accMax_new = zeros(3, 1);

vMax_new(1) = ((qFinish(1)-qStart(1))/(tf_new - ta_new))
accMax_new(1) = vMax_new(1)/ta_new
 
vMax_new(2) = ((qFinish(2)-qStart(2))/(tf_new - ta_new))
accMax_new(2) = vMax_new(2)/ta_new

vMax_new(3) = ((qFinish(3)-qStart(3))/(tf_new - ta_new));
accMax_new(3) = vMax_new(3)/ta_new;


% joint 1 - coefficients:

% t0 --> ta:
a10 = qStart(1);
a11 = vStart(1);
a12 = 0.5 * accMax_new(1);

% ta --> tf-ta:
a20 = qStart(1) + 0.5 * accMax_new(1) * ta_new^2 - vMax_new(1) * ta_new;
a21 = vMax_new(1);

% tf-ta --> tf:
a30 = qFinish(1) - 0.5 * accMax_new(1) * tf_new^2;
a31 = accMax_new(1) * tf_new;
a32 = -0.5 * accMax_new(1);

% joint 2 - coefficients:
% t0 --> ta:
b10 = qStart(2);
b11 = vStart(2);
b12 = 0.5 * accMax_new(2);

% ta --> tf-ta:
b20 = qStart(2) + 0.5 * accMax_new(2) * ta_new^2 - vMax_new(2) * ta_new;
b21 = vMax_new(2);

% tf-ta --> tf:
b30 = qFinish(2) - 0.5 * accMax_new(2) * tf_new^2;
b31 = accMax_new(2) * tf_new;
b32 = -0.5 * accMax_new(2);


% % joint 3 - coefficients:
% t0 --> ta:
c10 = qStart(3);
c11 = vStart(3);
c12 = 0.5 * accMax_new(3);

% ta --> tf-ta:
c20 = qStart(3) + 0.5 * accMax_new(3) * ta_new^2 - vMax_new(3) * ta_new;
c21 = vMax_new(3);

% tf-ta --> tf:
c30 = qFinish(3) - 0.5 * accMax_new(3) * tf_new^2;
c31 = accMax_new(3) * tf_new;
c32 = -0.5 * accMax_new(3);


t = 0:delta_t:tf_new;
q1 = (a10+a11.*t+a12.*t.^2).*(t<=ta_new)...
    +(a20+a21.*t).*(t>ta_new).*(t<=(tf_new-ta_new))...
    +(a30+a31.*t+a32.*t.^2).*(t>(tf_new-ta_new)).*(t<=tf_new);
v1 = (a11+2*a12.*t).*(t<=ta_new)...
    +(a21).*(t>ta_new).*(t<=(tf_new-ta_new))...
    +(a31+2*a32.*t).*(t>(tf_new-ta_new)).*(t<=tf_new);
acc1 = (2*a12).*(t<=ta_new)...
    +(0).*(t>ta_new).*(t<=(tf_new-ta_new))...
    +(2*a32).*(t>(tf_new-ta_new)).*(t<=tf_new);

q2 = (b10+b11.*t+b12.*t.^2).*(t<=ta_new)...
    +(b20+b21.*t).*(t>ta_new).*(t<=(tf_new-ta_new))...
    +(b30+b31.*t+b32.*t.^2).*(t>(tf_new-ta_new)).*(t<=tf_new);
v2 = (b11+2*b12.*t).*(t<=ta_new)...
    +(b21).*(t>ta_new).*(t<=(tf_new-ta_new))...
    +(b31+2*b32.*t).*(t>(tf_new-ta_new)).*(t<=tf_new);
acc2 = (2*b12).*(t<=ta_new)...
    +(0).*(t>ta_new).*(t<=(tf_new-ta_new))....
    +(2*b32).*(t>(tf_new-ta_new)).*(t<=tf_new);

q3 = (c10+c11.*t+c12.*t.^2).*(t<=ta_new)...
    +(c20+c21.*t).*(t>ta_new).*(t<=(tf_new-ta_new))...
    +(c30+c31.*t+c32.*t.^2).*(t>(tf_new-ta_new)).*(t<=tf_new);
v3 = (c11+2*c12.*t).*(t<=ta_new)...
    +(c21).*(t>ta_new).*(t<=(tf_new-ta_new))...
    +(c31+2*c32.*t).*(t>(tf_new-ta_new)).*(t<=tf_new);
acc3 = (2*c12).*(t<=ta_new)...
    +(0).*(t>ta_new).*(t<=(tf_new-ta_new))....
    +(2*c32).*(t>(tf_new-ta_new)).*(t<=tf_new);


% Visualisation

figure
plot(t,q1)
hold on
plot(t,q2)
grid on
plot(t,q3)
grid on
title('position vs time')
legend('j_1', 'j_2','j_3')
axis([0 tf_new -inf inf])

figure
plot(t,v1)
hold on
plot(t,v2)
hold on
plot(t,v3)
title('velocity vs time')
legend('j_1','j_2','j_3')
grid on
axis([0 tf_new -inf inf])

figure
plot(t,acc1)
hold on
plot(t,acc2)
hold on
plot(t,acc3)
title('acceleration vs time')
legend('j_1','j_2','j_3')
grid on
axis([0 tf_new -inf inf])





