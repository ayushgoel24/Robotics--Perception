function X = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)
%% LinearTriangulation
% Find 3D positions of the point correspondences using the relative
% position of one camera from another
% Inputs:
%     C1 - size (3 x 1) translation of the first camera pose
%     R1 - size (3 x 3) rotation of the first camera pose
%     C2 - size (3 x 1) translation of the second camera
%     R2 - size (3 x 3) rotation of the second camera pose
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Outputs: 
%     X - size (N x 3) matrix whos rows represent the 3D triangulated
%       points

N = size(x1, 1);
X = zeros(N, 3);

A = K * [R1 -R1*C1];
B = K * [R2 -R2*C2];

for i=1:N
    x_1 = [x1(i, :) 1];
    x_2 = [x2(i, :) 1];
    
    skew1 = Vec2Skew(x_1);
    skew2 = Vec2Skew(x_2);

    M = [skew1 * A; skew2 * B];
    
    [~, ~, m3] = svd(M);
    
    M_norm = m3(:, end) / m3(end, end);
    X(i, :) = M_norm(1:3, :)';
end
end