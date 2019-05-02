function [tf] = dh_transform(a, d, alpha, theta)
%DH_TRANSFORM Computes the DH transform from four DH parameters
%     a is distance between previous Z and current Z measured along current X.
% 
%     d is distance between previous Origin and the intersection between current X and previous Z mearured along previous Z.
% 
%     alpha is the angle between previous Z and current Z measured on the place normal to current X.
% 
%     theta  is the angle between previous X and current X measured on the plane normal to previous Z.

    rotz_th = basic_homogeneous_rotation(theta, 'z');
    transz_d = basic_homogeneous_translation(d, 'z');
    transx_a = basic_homogeneous_translation(a, 'x');
    rotx_al = basic_homogeneous_rotation(alpha, 'x');
    
    tf = rotz_th*transz_d*transx_a*rotx_al;

end

