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

rotation = [a, b, c; e, f, g; i, j, k];
translation = [d; h; l];
ref_coord = [z * u_ref; z * v_ref; z];

ref_transform = rotation * ref_coord + translation;

res = (ref_transform(1)/ref_transform(3) - u_curr)^2 + (ref_transform(2)/ref_transform(3) - v_curr)^2;

d_res = (diff(res))

%trying to minimize the derivative
%res = d_res^2;
%d_res = diff(res);



%evaluate
res = subs(res, rotation, [1, 0, 0; 0, 1, 0; 0, 0, 1]);
res = subs(res, translation, [-0.05; 0; 0]);
res = subs(res, [u_ref, v_ref], [0, 0]);
res = subs(res, [u_curr, v_curr], [-0.01, 0.1])

d_res = subs(d_res, rotation, [1, 0, 0; 0, 1, 0; 0, 0, 1]);
d_res = subs(d_res, translation, [-0.05; 0; 0]);
d_res = subs(d_res, [u_ref, v_ref], [0, 0]);
d_res = subs(d_res, [u_curr, v_curr], [-0.01, 0.1])


%test gauss newton (but 1-d)

optim_z = 0.9 % initial guess
last_optim_z = optim_z
current_res = double(subs(res, z, optim_z))

for it = (1:10)
  last_optim_z = optim_z
  optim_z = optim_z - (1 / double(subs(d_res, z, optim_z))^2) * double(subs(d_res, z, optim_z)) * double(subs(res, z, optim_z))
  
  if(double(subs(res, z, optim_z)) > current_res)
  break;
  end
  
  current_res = double(subs(res, z, optim_z))

end


printf('DONE')
last_optim_z
current_res