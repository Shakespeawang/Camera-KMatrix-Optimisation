function [ T_cw ] = FillImage( T_ow, GridWidth, CameraStructure )
%FILLIMAGE Function positions camera in such a way that it's image is
%entirely filled by a grid of width 'GridWidth' which is positioned and
%oriented according to the homogeneous transformation T_ow.

%The camera's KMatrix, chip width and chip height are passed in
%CameraStructure

%The camera is oriented to be viewing via a vector nearly orthogonal to the
%plane of the grid

%Function returns a 4x4 matrix T_cw, defining a homogeneous transformation
%from camera to world coordinates

%Check the validity of the inputs
TestTransformMatrix(T_ow, 'T_ow')

%Define the extreme corners of the grid in its own coordinate system to see 
%when it fills the camera's view
GridCorners = [GridWidth/2, -GridWidth/2, -GridWidth/2, GridWidth/2;
               GridWidth/2, GridWidth/2, -GridWidth/2, -GridWidth/2;
                  0,            0,             0,           0      ;
                  1,            1,             1,           1];

%Convert GridCorners to world coordinates
GridCorners = T_ow*GridCorners;

%Extract the 3-D coordinates to plot them later 
GridCorners_3D = GridCorners(1:3, :);

%Define an initial distance from which the camera views the object (this
%will be incrementally reduced until the grid fills the camera's view)
CameraDistance = 10000;

%Use a 'while' loop with a flag 'InsideImage' which sets to zero once the
%camera is in a suitable position

InsideImage = 1;

while InsideImage == 1
    InsideImage = 0;
    
    %Position the camera in space so it looks almost normally at the grid
    %from a distance CameraDistance 
    T_cw = PositionCamera2(T_ow, CameraDistance);
    
    %Convert GridCorners to camera coordinates (in the unit camera plane)
    %The transformation from world to camera coordinates is the inverse of
    %the transformation from camera to world coordinates.
    %T_wc = (T_cw)^(-1)
    UnitCorners = T_cw\GridCorners;
    
    %Remove the 4th homogeneous point from GridCorners and use the camera's 
    %KMatrix to transform the resulting matrix into 2-D homogeneous Points 
    %on the camera's sensor chip
    Corners = CameraStructure.KMatrix*UnitCorners(1:3,:);
    
    %Scale each Point in Corners by its homogeneous scaler to recover
    %2-D coordinates in 'u' and 'v' steps on the camera's sensor chip 
    s = size(Corners, 2);
    
    %Assign space for the 2-D coordinates in [u,v]
    GridCorners_uv = zeros(2, s);
    
    %Extract the chip width and chip height, for clarity, from
    %CameraStructure
    ChipWidth = CameraStructure.ChipWidth;
    ChipHeight = CameraStructure.ChipHeight;
    
    for i = 1:s 
        GridCorners_uv(:, i) = Corners(1:2, i)/Corners(3, i);
        
        %Check to see if each grid corner is within the boundaries of the
        %camera's sensor chip. If not, set InsideImage to 1, making the
        %program iterate the process again, with a shorter
        %CameraDistance
        if GridCorners_uv(1, i) > 0 && GridCorners_uv(1,i) < ChipWidth - 1
            InsideImage = 1;
        end
        if GridCorners_uv(2, i) > 0 && GridCorners_uv(2,i) < ChipHeight - 1
            InsideImage = 1;
        end
    end
        
    if InsideImage == 1
        CameraDistance = 0.9*CameraDistance;
    end 
end

end

