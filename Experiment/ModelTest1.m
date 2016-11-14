%create all variables
syms x y z dx dy dz q0 q1 q2 q3 bgx bgy bgz bax bay baz ax ay az wx wy wz dt


%create the state
state = [x; y; z; dx; dy; dz; q0; q1; q2; q3; bgx; bgy; bgz; bax; bay; baz];

%this rotates the acceleration into the world coordinate frame
newA = [(1-2*q2^2-2*q3^2), 2*(q1*q2+q0*q3), 2*(q1*q3-q0*q2); 
        2*(q1*q2-q0*q3), (1-2*q1^2-2*q3^2), 2*(q2*q3+q0*q1);
        2*(q1*q3+q0*q2), 2*(q2*q3-q0*q1), (1-2*q1^2-2*q2^2)] * [(ax+bax); (ay+bay); (az+baz)];

%create the delta quaternion
w_mag = sqrt((wx+bgx)^2 + (wy+bgy)^2 + (wz+bgz)^2);

%IMPORTANT--- if the omega values are 0 use this
dq0 = 1;
dq1 = 0;
dq2 = 0;
dq3 = 0;

%dq0 = cos(0.5 * w_mag * dt);
%dq1 = (2 * wx / w_mag) * sin(0.5 * w_mag * dt);
%dq2 = (2 * wy / w_mag) * sin(0.5 * w_mag * dt);
%dq3 = (2 * wz / w_mag) * sin(0.5 * w_mag * dt);

dq_mag = sqrt(dq0^2+dq1^2+dq2^2+dq3^2);
dq0 = dq0 / dq_mag;
dq1 = dq1 / dq_mag;
dq2 = dq2 / dq_mag;
dq3 = dq3 / dq_mag;
    
hx = [x + dx*dt + 0.5*newA(1)*dt*dt;
      y + dy*dt + 0.5*newA(2)*dt*dt;
      z + dz*dt + 0.5*newA(3)*dt*dt;
      dx + newA(1)*dt;
      dy + newA(2)*dt;
      dz + newA(3)*dt;
      q0*dq0 - q1*dq1 - q2*dq2 - q3*dq3;
      q0*dq1 + q1*dq0 - q2*dq3 + q3*dq2;
      q0*dq2 + q1*dq3 + q2*dq0 - q3*dq1;
      q0*dq3 - q1*dq2 + q2*dq1 + q3*dq0;
      bgx;
      bgy;
      bgz;
      bax;
      bay;
      baz];
  
H = jacobian(hx, state)

%tempH = subs(H, state, [1;0;0;0;0;0;1;0;0;0;0;0;0;0;0;0]);

%tempH = subs(tempH, [ax;ay;az;wx;wy;wz;dt], [0;0;0;0;0;0;0.02])


