function [RegressorRows, DataVectorRows] = ...
    HomogRowPair( SamplePoint )
%HOMOGROWPAIR Function uses an [x,y] point on an object, and its noisy
%[u,v] correspondence on the camera's sensor chip, to build two rows of both
%a Regresson matrix and a data vector. Both these arrays will be used to
%estimate a homography between the object's plane and the camera's sensor
%chip

%The [x,y] and [u,v] coordinates are stored in the vector SamplePoint

%The Regressor matrix and data vector are related as: 
% Regressor*HomographyVector = DataVector (where HomographyVector is the
% elements of the homography rearranged as a vector)
 
%Assign space for the output arguments
RegressorRows = zeros(2, 8);
DataVectorRows = zeros(1, 2); 

%Extract data from SamplePoint for clarity;
u = SamplePoint(1);
v = SamplePoint(2);
x = SamplePoint(3);
y = SamplePoint(4);

%Construct RegressorRows
RegressorRows(1,:) = [x, y, 1, 0, 0, 0, -u*x, -u*y];
RegressorRows(2,:) = [0, 0, 0, x, y, 1, -v*x, -v*y];

%Construct DataVectorRows
DataVectorRows = [u; v];

end

