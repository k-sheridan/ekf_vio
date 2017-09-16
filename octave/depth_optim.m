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
syms u_ref, v_ref, u_curr, v_curr
syms z

