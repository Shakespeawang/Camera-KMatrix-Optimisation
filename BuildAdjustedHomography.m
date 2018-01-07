function [ KMatrix, Rotation, Translation ] = BuildAdjustedHomography(...
    P )
%BUILDADJUSTEDHOMOGRAPHY A specific fucntion which, given a vector P
%containing parameters within a homography, will build the components of
%the homography. The homography is broken down into a KMatrix of a camera
%and a perspectivity, which is in itself broken down into a Rotation and
%Translation 

%Check that the input vector is of correct length

if length(P) ~= 11
    error('Adjusted homography cannot be built for Jacobian derivatives \n input vector incorrect length')
end

KMatrix = [P(1), P(2), P(3);
            0,   P(4), P(5);
            0,    0,    1];
        
Rotation = [P(6); P(7); P(8)];

Translation = [P(9); P(10); P(11)];

end

