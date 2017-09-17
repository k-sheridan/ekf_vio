% transform from the current camera to the reference camera
% [R|t] = [a b c d
%          e f g h
%          i j k l]
%
% [u_ref, v_ref] = homogenous coordinates of the reference pixel
% [u_curr, v_curr] = homogenous coordinates of the current pixel
%
% z = the depth the the reference pixel (to be optimized 1-d)

syms a b c d e f g h i j k l
syms u_ref v_ref u_curr v_curr
syms z

syms qw qx qy qz

rotation = [1 - 2*qy^2 - 2*qz^2,    2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw;
            2*qx*qy + 2*qz*qw,    1 - 2*qx^2 - 2*qz^2,    2*qy*qz - 2*qx*qw;
            2*qx*qz - 2*qy*qw,    2*qy*qz + 2*qx*qw,    1 - 2*qx^2 - 2*qy^2];
translation = [d; h; l];


%E = rotation * [0, -l, h; l, 0, -d; -h, d, 0]

% attempt orthoganal projection onto epipole

epipole = translation
meter_feature_dir = rotation * [u_ref;v_ref;1] + translation;

epiline = meter_feature_dir - epipole;
measured_epiline = [u_curr;v_curr;1] - epipole;

projected_measurement3 = (dot(measured_epiline, epiline) / dot(epiline, epiline)) * epiline + epipole

projected_measurement = [(projected_measurement3(1)/projected_measurement3(3)); (projected_measurement3(2)/projected_measurement3(3))]

temp = subs(projected_measurement, rotation, [1, 0, 0; 0, 1, 0; 0, 0, 1]);
temp = subs(temp, translation, [-0.05; 0; 0.0]);
temp = subs(temp, [u_ref, v_ref], [0, 0]);
double(subs(temp, [u_curr, v_curr], [-0.05, 0.0]))

u_curr = projected_measurement(1);
v_curr = projected_measurement(2);


A = [(rotation * [u_ref;v_ref;1]), [u_curr;v_curr;1]];

AtA = A' * A;

depth2 = - inv(AtA)*A'*translation;

soln = simplify(depth2(1))

z = abs(soln)

ccode(z)


soln = subs(soln, rotation, [1, 0, 0; 0, 1, 0; 0, 0, 1]);
soln = subs(soln, translation, [-0.05; 0; 0.0001]);
soln = subs(soln, [u_ref, v_ref], [0, 0]);
double(subs(soln, [u_curr, v_curr], [-0.05, 0.01]))



