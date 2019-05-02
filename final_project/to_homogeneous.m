function [result] = to_homogeneous(rotation_or_translation)
%TO_HOMOGENEOUS Convert translation vector or rotation matrix
%to homogeneous form
    dims = size(rotation_or_translation);
    result = sym(eye(4));
    if isequal(dims,[3, 3])
        rotation = rotation_or_translation;
        result(1:3, 1:3) = rotation;
    else
        translation = rotation_or_translation;
        result(1:3, 4) = translation;
    end
end

