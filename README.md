# TrajectoryPlanning
IK:
  
  q1 = atan2(Y, X)

  AB = sqrt((X^2 + Y^2) + (Z - L(1))^2);

  q21 = atan2(Z - L(1), (X^2 + Y^2));
  q22 = acos((L(2)^2 + AB^2 - L(3)^2)/(2 * L(2) * AB));
  q2 = q21 + q22

  q3 = vpa(pi - acos((L(3)^2 + L(2)^2 - AB^2) / (2 * L(2) * L(3))))

# Visualisation
![Pic](https://user-images.githubusercontent.com/27694775/96355680-dfeb5400-10ec-11eb-82c9-1f7e8e7cb44f.png)
