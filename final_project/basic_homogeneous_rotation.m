function [result] = basic_homogeneous_rotation(angle,axis)
%BASIC_HOMOGENEOUS_ROTATION Returns a basic homogeneous rotation matirx
%  axis is 'x', 'y', or 'z'
% angle is a number or symoblic
    axis = lower(axis);
    t = angle;
    if axis == 'x'
        Rx = [1 0 0; 0 cos(t) -sin(t); 0 sin(t) cos(t)];
        result = Rx;
    elseif axis == 'y'
        Ry = [cos(t) 0 sin(t); 0 1 0; -sin(t) 0 cos(t)];
        result = Ry;
    elseif axis == 'z'
        Rz = [cos(t) -sin(t) 0; sin(t) cos(t) 0; 0 0 1];
        result = Rz;
    else
        error('axis must be x, y, or z for basic_homogeneous transform')
    end
    result = to_homogeneous(result);
end

