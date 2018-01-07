function [ UV_star_points ] = PredictSensorPoints( Correspond, CorrespondIndex,...
    Rotation,Translation, KMatrix )
%PREDICTSENSORPOINTS Function transforms a set of [x,y] points in an
%object's frame into a set of [u*,v*] points on a camera's sensor chip as
%predicted by a perspectivity and the camera's KMatrix (together forming a
%homography)

%The points in the object's frame are stored within a Correspond matrix
%which also contains the the true 'sensed' [u,v] points on the camera which
%correspond to the [x,y] points in the form: [u,v,x,y]^T.

%Only some of the points in Correspond are in the consensus set for the
%homography, the column indices of these points are stored in
%CorrespondIndex.

%The perspectivity is defined by a rotation(in Shifted-Angle axis form) and
%a translation vector

%Begin by extracting the number of points in the consensus set
k = nnz(CorrespondIndex);

%Convert the Shifted-Angle Axis rotation into a 3x3 rotation matrix
RotationAngle = norm(Rotation);
RotationAxis = Rotation/RotationAngle;

RotationMatrix = RodriguesRotation(RotationAxis,RotationAngle);

%Build a homogeneous transformation to transform [x,y,0,1] homogeneous
%Points in the object's frame to Points in the camera's unit plane.
T = [RotationMatrix, Translation; 0, 0, 0, 1];

%Assign space for a matrix containing all the [x y 0 1] Points (relative to
%the object's frame) within the consensus set
XYPoints = zeros(4, k);
XYPoints(4,:) = ones(1,k);

%Fill XYPoints
for j=1:k
    XYPoints(1:2,j) = Correspond(3:4, CorrespondIndex(j));
    
    %Function only works with break here. Why? 
    if j==k
        break 
    end
end

%Transform the matrix of XYPoints into [u*,v*] predicted points

%First transform into the unit plane
Points = T*XYPoints;

%Extract the 3-dimensional coordinates (now in the camera cooridinate
%system) from ObjectPoints
Points_3D = Points(1:3, :);

%Multiply by the K-Matrix to turn these 3-D points into 2-D homogeneous
%Points
ObjectPoints_2DH = KMatrix*Points_3D;

%Assign space for the predicted points [u*,v*] 
UV_star_points = zeros(2, k);

%Scale each 2-D homogeneous Point by its homogeneous multiplier to find the
%predicted point on the camera's sensor chip
for i = 1:k;
    UV_star_points(1:2, i) = ObjectPoints_2DH(1:2, i)/ObjectPoints_2DH(3,i);
end

end

