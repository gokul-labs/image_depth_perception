function depthM = get_depth(dispM, K1, K2, R1, R2, t1, t2)
% GET_DEPTH creates a depth map from a disparity map (DISPM).
c1 = -inv(R1) * inv(K1) * (K1 * t1);
c2 = -inv(R2) * inv(K2) * (K2 * t2);

% assume that b = ∥c1 − c2∥ (i.e., distance between optical centers) and f = K1(1, 1)
b = norm(c1 - c2);
f = K1(1, 1);
depthM = b * f ./ dispM;

% let depthM(y, x) = 0 whenever dispM(y, x) = 0 to avoid dividing by 0.
depthM(dispM == 0) = 0;