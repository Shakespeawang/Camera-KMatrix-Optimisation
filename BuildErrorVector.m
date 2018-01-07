function [ Error ] = BuildErrorVector( KMatrix, Rotation, Translation, ...
    Correspond, CorrespondIndex)
%BUILDERRORVECTOR Function builds an error vector between points on a
%camera's sensor chip, predicted by a homography, and the corresponding
%points 'actually sensed' on the chip. The vector has length equal to twice
%the number of points within the consensus set that is passed as an
%argument

%KMatrix, Rotation and Translation together build the current estimate of
%the homography between the object's plane and the camera's sensor chip.
%They will be used to calulate the 'predicted points'. The rotation is in
%Shifted-Angle Axis form.

%Correspond contains vectors of the form [u v x y]^T which are
%correspondences between [x,y] points on the object and their corresponding
%(noisy) [u,v] points 'sensed' on the camera's chip.

%CorrespondIndex contains the column indices of the points in the
%Correspond matrix which fall within the consensus set for the homography

%Check the validity of the inputs 
s = size(KMatrix);
if s(1) ~=3 || s(2) ~=3
    error('KMatrix has invalid dimensions')
end

%Check KMatrix is upper diagonal
if KMatrix(2,1) ~=0 || KMatrix(3,1) ~=0 ...
        || KMatrix(3,2) ~=0
    error('KMatrix is not upper diagonal')
end

%Make sure the rotation and translation are working on the same sized space
if length(Translation) ~= length(Rotation)
    error('Rotation and Translation in BuildErrorVector are incompatable')
end

%END OF CHECKS ON INPUTS

%Begin by extracting the number of points in the consensus set
k = nnz(CorrespondIndex);

%Assign space for the error vector
Error = zeros(2*k, 1);

%Assign space for a matrix containing the all of the true [u,v] points
%(taken from Correspond) which are in the consensus set
UVpoints = zeros(2,k);

%Fill XYPoints and UVPoints
for j=1:k
    UVpoints(1:2,j) = Correspond(1:2, CorrespondIndex(j));
    
    %Function only works with break here. Why? 
    if j==k
        break 
    end
    
end

%Compute the predicted [u*,v*] points
UV_star_points = PredictSensorPoints(Correspond, CorrespondIndex, Rotation...
                        ,Translation, KMatrix);

%Find the errors between the predicted [u*,v*] points computed above and the true
%[u,v] points

%The error vector is contstructed as [UError, VError]
Error(1:k) = UV_star_points(1,:) - UVpoints(1,:);
Error(k+1:2*k) = UV_star_points(2,:) - UVpoints(2,:);

end

