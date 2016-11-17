function [newState] = stateTransition(state, measurement, dt)
    
q = quatnormalize([state(7) state(8) state(9) state(10)]);

x = state(1);
y = state(2);
z = state(3);
dx = state(4);
dy = state(5);
dz = state(6);
q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);
bax = state(11);
bay = state(12);
baz = state(13);
bgx = state(14);
bgy = state(15);
bgz = state(16);

ax = measurement(1);
ay = measurement(2);
az = measurement(3);
wx = measurement(4);
wy = measurement(5);
wz = measurement(6);

newState = [(((ay + bay)*(2*q0*q3 + 2*q1*q2))/2 - ((ax + bax)*(2*q2^2 + 2*q3^2 - 1))/2 - ((az + baz)*(2*q0*q2 - 2*q1*q3))/2)*dt^2 + dx*dt + x;
      (((az + baz)*(2*q0*q1 + 2*q2*q3))/2 - ((ax + bax)*(2*q0*q3 - 2*q1*q2))/2 - ((ay + bay)*(2*q1^2 + 2*q3^2 - 1))/2)*dt^2 + dy*dt + y;
      (((ax + bax)*(2*q0*q2 + 2*q1*q3))/2 - ((az + baz)*(2*q1^2 + 2*q2^2 - 1))/2 - ((ay + bay)*(2*q0*q1 - 2*q2*q3))/2)*dt^2 + dz*dt + z;
       dx - dt*((ax + bax)*(2*q2^2 + 2*q3^2 - 1) - (ay + bay)*(2*q0*q3 + 2*q1*q2) + (az + baz)*(2*q0*q2 - 2*q1*q3));
       dy - dt*((ay + bay)*(2*q1^2 + 2*q3^2 - 1) + (ax + bax)*(2*q0*q3 - 2*q1*q2) - (az + baz)*(2*q0*q1 + 2*q2*q3));
       dz - dt*((az + baz)*(2*q1^2 + 2*q2^2 - 1) - (ax + bax)*(2*q0*q2 + 2*q1*q3) + (ay + bay)*(2*q0*q1 - 2*q2*q3));
       -(2*q1*wx*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2) + 2*q2*wy*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2) + 2*q3*wz*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2) - q0*cos((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/(((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2)*(cos((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)^2 + (4*wx^2*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)^2)/((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2) + (4*wy^2*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)^2)/((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2) + (4*wz^2*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)^2)/((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2))^(1/2));
       (2*q0*wx*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2) + 2*q3*wy*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2) - 2*q2*wz*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2) + q1*cos((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/(((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2)*(cos((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)^2 + (4*wx^2*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)^2)/((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2) + (4*wy^2*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)^2)/((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2) + (4*wz^2*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)^2)/((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2))^(1/2));
       (2*q0*wy*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2) - 2*q3*wx*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2) + 2*q1*wz*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2) + q2*cos((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/(((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2)*(cos((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)^2 + (4*wx^2*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)^2)/((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2) + (4*wy^2*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)^2)/((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2) + (4*wz^2*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)^2)/((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2))^(1/2));
       (2*q2*wx*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2) - 2*q1*wy*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2) + 2*q0*wz*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2) + q3*cos((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/(((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2)*(cos((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)^2 + (4*wx^2*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)^2)/((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2) + (4*wy^2*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)^2)/((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2) + (4*wz^2*sin((dt*((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2)^(1/2))/2)^2)/((bgx + wx)^2 + (bgy + wy)^2 + (bgz + wz)^2))^(1/2));
       bax;
       bay;
       baz;
       bgx;
       bgy;
       bgz;];

end