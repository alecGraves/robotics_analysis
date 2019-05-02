test(to_homogeneous([1, 2, 3]'), sym([1, 0, 0, 1; 0, 1, 0, 2; 0, 0, 1, 3; 0, 0, 0, 1]), 1);
test(to_homogeneous([1, 2, 3; 4, 5, 6; 7, 8, 9]), sym([1, 2, 3, 0; 4, 5, 6, 0; 7, 8, 9, 0; 0, 0, 0, 1]), 2);

syms x
result = basic_homogeneous_rotation(x, 'z');
expected = [ cos(x), -sin(x), 0, 0;
            sin(x),  cos(x), 0, 0;
             0,       0, 1, 0;
             0,       0, 0, 1];
test(result, expected, 3)

result = basic_homogeneous_translation(x, 'z');
expected = [ 1, 0, 0, 0;
             0, 1, 0, 0;
             0, 0, 1, x;
             0, 0, 0, 1];
test(result, expected, 4)

pi = sym(pi);

syms t1 d2 d3
joint_vars = [t1 d2 d3];
arrayfun(@(var) assume(var,'real'), joint_vars);

h0_1 = dh_transform(0, 1, 0, t1);
h1_2 = dh_transform(1, d2, 0, pi/2);
h2_3 = dh_transform(d3, 0, 0, 0);
h0_3 = h0_1*h1_2*h2_3
get_jacobian(h0_3, [t1, d2, d3])

function [] = test(result, expected, n)
    if isequal(result, expected)
        disp(strcat(['test ', int2str(n), ' passed']));
    else
        warning(strcat(['test ', int2str(n), ' FAILED']));
    end
end