function [jacobian] = get_jacobian(forward_homogeneous,joint_vars)
%GET_JACOBIAN Computes the jacobian
%   Uses calculus to determine how each joint 
%var will effect end-effector motion
    translation = forward_homogeneous(1:3, 4);
    rotation = forward_homogeneous(1:3, 1:3);
    num_vars = length(joint_vars);
    jacobian = sym(zeros(6, num_vars));
    for i = 1:num_vars
        var = joint_vars(i);
        translation_jacobian = diff(translation, var);
        rotation_jacobian = skew2vec(diff(rotation, var)*rotation');

        jacobian(1:3, i) = translation_jacobian;
        jacobian(4:6, i) = rotation_jacobian;
    end

    jacobian = simplify(jacobian);
end

function [vec] = skew2vec(mat)
%SKEW2VEC Converts 3d skew-symmetric matrix to vector [21, 02, 10]
    vec0 = mat(3, 2);
    vec1 = mat(1, 3);
    vec2 = mat(2, 1);
    vec = [vec0, vec1, vec2];
end