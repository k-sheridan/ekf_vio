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
%syms w_mag

%IMPORTANT--- if the omega values are 0 use this
%dq0 = 1;
%dq1 = 0;
%dq2 = 0;
%dq3 = 0;

dq0 = cos(0.5 * w_mag * dt);
dq1 = (2 * wx / w_mag) * sin(0.5 * w_mag * dt);
dq2 = (2 * wy / w_mag) * sin(0.5 * w_mag * dt);
dq3 = (2 * wz / w_mag) * sin(0.5 * w_mag * dt);

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
  
% syms omega_mag s1 s2 s3 s4 s5 s6 s7 s8 s9 s10 s11 s12 s13 s14 s15 s16 s17 s18 s19 s20 s21
% syms s22 s23 s24 s25 s26 s27 s28 s29 s30 s31 s32 s33 s34 s35 s36 s37 s38 s39 s40 s41 s42
% H = subs(simplify(jacobian(hx, state)), [bax;bay;baz;bgx;bgy;bgz], [0;0;0;0;0;0]);
% H = subs(H, (wx^2 + wy^2 + wz^2)^(1/2), omega_mag);
% H = simplify(subs(H, sin((dt*omega_mag)/2), s1));
% H = simplify(subs(H, ((dt*omega_mag)/2), s2));
% H = simplify(subs(H, ((omega_mag^2*cos(s2)^2 + 4*s1^2*wx^2 + 4*s1^2*wy^2 + 4*s1^2*wz^2)/omega_mag^2)^(1/2), s4));
% H = simplify(subs(H, cos(s2)^3, s5));
% H = simplify(subs(H, cos(s2), s6));
% H = simplify(subs(H, dt*s1^2*s6, s7));
% H = simplify(subs(H, s1*s6^2, s8));
% H = simplify(subs(H, 2*omega_mag, s9));
% H = simplify(subs(H, s1^3, s10));
% H = simplify(subs(H, (omega_mag*s4), s11));
% H = simplify(subs(H, (2*s1*wx), s12));
% H = simplify(subs(H, s6/s4, s13));
% H = simplify(subs(H, 2*dt*omega_mag, s14));
% H = subs(H, (omega_mag*s11^3), s15);
% H = simplify(H);
% H = subs(H, [wx^3;wy^3;wz^3], [s16;s17;s18]);
% H = simplify(H);
% H = subs(H, [wx^2;wy^2;wz^2], [s19;s20;s21]);
% H = simplify(H);
% H = subs(H, 2*dt*omega_mag, s22);
% H = simplify(H);
% H = subs(H, dt*q0, s23);
% H = simplify(H);
% H = subs(H, dt*q1, s24);
% H = simplify(H);
% H = subs(H, dt*q2, s25);
% H = simplify(H);
% H = subs(H, dt*q3, s26);
% H = simplify(H);
% H = subs(H, 2*omega_mag*s8, s27);
% H = simplify(H, 1000);
% H = subs(H, -2*omega_mag, s28);
% H = simplify(H);
% H = subs(H, [q0*s7,q1*s7,q2*s7,q3*s7], [s29,s30,s31,s32]);
% H = simplify(H);
% H = subs(H, [s5*s16,s5*s17,s5*s18], [s33,s34,s35]);
% H = simplify(H);
% H = subs(H, dt^2, s36);
% H = simplify(H);
% H = subs(H, 2*dt, s37);
% H = simplify(H);
% H = subs(H, s1^2*s6, s38);
% H = simplify(H);
% H = subs(H, [s5*s19,s5*s20,s5*s21], [s39,s40,s41]);
% H = simplify(H);
% H = subs(H, s8*s28, s42);
% H = simplify(H)

syms s1 s2 s3 s4 s5 s6 s7 s8 s9 s10 s11 s12 s13 s14 s15 s16 s17 s18 s19 s20
syms s21 s22 s23 s24 s25 s26 s27 s28 s29 s30 s31 s32 s33 s34 s35 s36 s37 s38
syms s39 s40 s41 s42 s43 s44 s45 s46 s47 s48 s49 s50
syms s51 s52 s53 s54 s55 s56 s57 s58 s59 s60 s61 s62 s63 s64 s65 s66 s67 s68 s69 s70
H = simplify(subs(jacobian(hx, state), [bax;bay;baz;bgx;bgy;bgz], [0;0;0;0;0;0]));
[H, s1] = subexpr(H, s1)
H = simplify(H);
[H, s2] = subexpr(H, s2)
H = simplify(H);
[H, s3] = subexpr(H, s3)
H = simplify(H);
[H, s4] = subexpr(H, s4)
H = simplify(H);
[H, s5] = subexpr(H, s5)
H = simplify(H);
[H, s6] = subexpr(H, s6)
H = simplify(H);
[H, s7] = subexpr(H, s7)
H = simplify(H);
[H, s8] = subexpr(H, s8)
H = simplify(H);
[H, s9] = subexpr(H, s9)
H = simplify(H);
[H, s10] = subexpr(H, s10)
H = simplify(H);
[H, s11] = subexpr(H, s11)
H = simplify(H);
[H, s12] = subexpr(H, s12)
H = simplify(H);
[H, s13] = subexpr(H, s13)
H = simplify(H);
[H, s14] = subexpr(H, s14)
H = simplify(H);
[H, s15] = subexpr(H, s15)
H = simplify(H);
[H, s16] = subexpr(H, s16)
H = simplify(H);
[H, s17] = subexpr(H, s17)
H = simplify(H);
[H, s18] = subexpr(H, s18)
H = simplify(H);
[H, s19] = subexpr(H, s19)
H = simplify(H);
[H, s20] = subexpr(H, s20)
H = simplify(H);
[H, s21] = subexpr(H, s21)
H = simplify(H);
[H, s22] = subexpr(H, s22)
H = simplify(H);
[H, s23] = subexpr(H, s23)
H = simplify(H);
[H, s24] = subexpr(H, s24)
H = simplify(H);
[H, s25] = subexpr(H, s25)
H = simplify(H);
[H, s26] = subexpr(H, s26)
H = simplify(H);
[H, s27] = subexpr(H, s27)
H = simplify(H);
[H, s28] = subexpr(H, s28)
H = simplify(H);
[H, s29] = subexpr(H, s29)
H = simplify(H);
[H, s30] = subexpr(H, s30)
H = simplify(H);
[H, s31] = subexpr(H, s31)
H = simplify(H);
[H, s32] = subexpr(H, s32)
H = simplify(H);
[H, s33] = subexpr(H, s33)
H = simplify(H);
[H, s34] = subexpr(H, s34)
H = simplify(H);
[H, s35] = subexpr(H, s35)
H = simplify(H);
[H, s36] = subexpr(H, s36)
H = simplify(H);
[H, s37] = subexpr(H, s37)
H = simplify(H);
[H, s38] = subexpr(H, s38)
H = simplify(H);
[H, s39] = subexpr(H, s39)
H = simplify(H);
[H, s40] = subexpr(H, s40)
H = simplify(H);
[H, s41] = subexpr(H, s41)
H = simplify(H);
[H, s42] = subexpr(H, s42)
H = simplify(H);
[H, s43] = subexpr(H, s43)
H = simplify(H);
[H, s44] = subexpr(H, s44)
H = simplify(H);
[H, s45] = subexpr(H, s45)
H = simplify(H);
[H, s46] = subexpr(H, s46)
H = simplify(H);
[H, s47] = subexpr(H, s47)
H = simplify(H);
[H, s48] = subexpr(H, s48)
H = simplify(H);
[H, s49] = subexpr(H, s49)
H = simplify(H);
[H, s50] = subexpr(H, s50)
H = simplify(H);
[H, s51] = subexpr(H, s51)
H = simplify(H);
[H, s52] = subexpr(H, s52)
H = simplify(H);
[H, s53] = subexpr(H, s53)
H = simplify(H);
[H, s54] = subexpr(H, s54)
H = simplify(H);
[H, s55] = subexpr(H, s55)
H = simplify(H);
[H, s56] = subexpr(H, s56)
H = simplify(H);
[H, s57] = subexpr(H, s57)
H = simplify(H);
[H, s58] = subexpr(H, s58)
H = simplify(H);
[H, s59] = subexpr(H, s59)
H = simplify(H);
[H, s60] = subexpr(H, s60)
H = simplify(H);
[H, s61] = subexpr(H, s61)
H = simplify(H);
[H, s62] = subexpr(H, s62)
H = simplify(H);
[H, s63] = subexpr(H, s63)
H = simplify(H);
[H, s64] = subexpr(H, s64)
H = simplify(H);
[H, s65] = subexpr(H, s65)
H = simplify(H);
[H, s66] = subexpr(H, s66)
H = simplify(H);
[H, s67] = subexpr(H, s67)
H = simplify(H);
[H, s68] = subexpr(H, s68)
H = simplify(H);
[H, s69] = subexpr(H, s69)
H = simplify(H);
[H, s70] = subexpr(H, s70)
H = simplify(H)

