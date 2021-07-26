function F = EstimateFundamentalMatrix(x1, x2)
%% EstimateFundamentalMatrix
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2

N = size(x1, 1);
mat = zeros(N,9);
mat(:,1) = x1(:,1) .* x2(:,1);
mat(:,2) = x1(:,2) .* x2(:,1);
mat(:,3) = x2(:,1);
mat(:,4) = x1(:,1) .* x2(:,2);
mat(:,5) = x1(:,2) .* x2(:,2);
mat(:,6) = x2(:,2);
mat(:,7) = x1(:,1);
mat(:,8) = x1(:,2);
mat(:,9) = 1;

[~, ~, m3] = svd(mat);

F = m3(:, end);
F = reshape(F, [3,3]);

[m1, m2, m3] = svd(F);
m2(3,3) = 0;
F = m1 * m2 * m3';

F = F/norm(F);

end


