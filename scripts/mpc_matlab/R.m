function res = R(theta)
% 2D rotation matrix
    res = [cos(theta), -sin(theta); sin(theta), cos(theta)];
end
