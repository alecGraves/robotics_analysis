function [result] = basic_homogeneous_translation(distance,axis)
%BASIC_HOMOGENEOUS_TRANSLATION returns a homogeneous translation matrix
%  axis is 'x', 'y', or 'z'
% distance is a number or symoblic
    axis = lower(axis);
    if axis == 'x'
        result = to_homogeneous([distance, 0, 0]);
    elseif axis == 'y'
        result = to_homogeneous([0, distace, 0]);
    elseif axis == 'z'
        result = to_homogeneous([0, 0, distance]);
    else
        error('axis must be x, y, or z for basic_homogeneous transform');
    end
end

