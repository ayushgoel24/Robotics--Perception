function [proj_points, t, R] = ar_cube(H,render_points,K)
%% ar_cube
% Estimate your position and orientation with respect to a set of 4 points on the ground
% Inputs:
%    H - the computed homography from the corners in the image
%    render_points - size (N x 3) matrix of world points to project
%    K - size (3 x 3) calibration matrix for the camera
% Outputs: 
%    proj_points - size (N x 2) matrix of the projected points in pixel
%      coordinates
%    t - size (3 x 1) vector of the translation of the transformation
%    R - size (3 x 3) matrix of the rotation of the transformation
% Written by Stephen Phillips for the Coursera Robotics:Perception course

% YOUR CODE HERE: Extract the pose from the homography
R1 = [H(:, 1), H(:, 2), cross(H(:,1), H(:, 2))];
[m1 m2 m3] = svd(R1);
norm_matrix = [1 0 0; 0 1 0; 0 0 det(m1*m3')];
R = m1 * norm_matrix * m3';
t = H(:, 3)/norm(H(:, 1));
if H(3, 3) < 0
    t = -t;
end

% YOUR CODE HERE: Project the points using the pose
proj_points = zeros(size(render_points, 1), 2);
x_var = K * (R * render_points' + t);
A = x_var(1,:) ./ x_var(3, :);
B = x_var(2,:) ./ x_var(3, :);
proj_points(:, 1) = A';
proj_points(:, 2) = B';

end
