%initialize the state vector
X = [0; 0; 0; 0; 0; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0];
%x, y, z, dx, dy, dz, qx, qy, qz, qw, bgx, bgy, bgz, bax, bay, baz
E = 10000 * eye(16, 16);
E(11:16, 11:16) = 1 * eye(6, 6);

