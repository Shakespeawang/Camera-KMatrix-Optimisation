function [ ] = TestTransformMatrix( T, string )
%TESTTRANSFORMMATRIX Function tests the integrity of a homogeneous
%transformation matrix. It ensures the matrix is of correct dimensions and
%has the correct bottom row

%T is the matrix to be checked
%'string' is a string input containing the name of the matrix

s = size(T);
if s(1) ~= 4 || s(2) ~= 4
    error('%s has invalid dimensions', string)
end

if T(4, :) ~= [0,0,0,1];
    error('%s is invalid', string)
end

end

