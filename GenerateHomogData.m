function [ HomogData ] = GenerateHomogData( nImages, T_ow, GridPoints,...
    CameraStructure, Scale, GridWidth )
%GENERATEHOMOGDATA Function returns a cell array, HomogData, containing
%data on 'nImage' homographies which are generated using a RANSAC
%estimation

%A single object, GridPoints, is viewed from nImage different locations.
%T_ow defines the position of the object in world coordinates
%CameraStructure contains the KMatrix, chip width and chip height of the
%camera viewing the object
%Scale is a structure containing the scaling factors for the grid and
%camera (in both the u and v directions)
%GridWidth the the length of one of the sides of the grid defined in
%GridPoints

%Homogata is a cell with nImage rows where cell(nImages, :) = 
%[Homography estimate, Matrix of Correspond Points, ...
%Array containing the column index of points in the consensus set]

HomogData = cell(nImages, 3);

for n=1:nImages
    
%Only store the returned homography and related data if the homography is
%non-zero (if the estimation worked). Use a flag, Estimating, to achieve
%this.
Estimating = 1;

    while Estimating ==1; 
        
        Estimating =0;
        
        %Position the camera so in such a way that it's image is entirely
        %filled by the grid. Do this by a homogeneous transformation T_cw,
        %defining the transformation from camera coordinates to world
        %coordinates
        T_cw = FillImage(T_ow, GridWidth, CameraStructure);

        %Next generate correspondences between points in the grid ([x,y]
        %in the grid's own system) and noisy measurements taken of these
        %points ([u,v] expressed in terms of steps in the camera's sensor
        %chip)

        %Define the noise power and the standard deviation of the noise (in
        %pixels)
        NoiseSD = sqrt(30);

        Correspond = BuildNoisyCorrespondences(GridPoints, T_ow, T_cw, ...
            CameraStructure, NoiseSD);

        %Add outliers to the noisy Correspond matrix. These are points that
        %do not agree with the camera model (with noise added).

        %Define the probability of an outlier occurring
        pOutlier = 0.05;

        Correspond = GenerateOutliers(Correspond, pOutlier, CameraStructure);

        %Now scale the points in Correspond according to Scale
        CameraScale = Scale.CameraScale;
        GridScale = Scale.GridScale;

        Correspond(1:2, :) = Correspond(1:2, :)*CameraScale -1;
        Correspond(3:4, :) = Correspond(3:4, :)*GridScale;

        %Run the RANSAC simulation to estimate the homography between the
        %object's plane and the camera's sensor chip

        %Define the maximum admissable error, in pixels, between points
        %predicted by an estimate of the homography and the true
        %coordinates
        MaxError = 5;

        %Scale the MaxError by the camera scaling factor
        MaxError = MaxError*CameraScale;
        
        %State how many times RANSAC schould iterate
        nRuns = 50;

        [EstimateH, BestCorrespondIndex]...
            = RansacHomog(Correspond, nRuns, MaxError);

        %If the returned homography is valid, store the data and display
        %the homography
        if EstimateH ~= zeros(3)
            HomogData{n,1} = EstimateH;
            HomogData{n,2} = Correspond;
            HomogData{n,3} = BestCorrespondIndex;
            EstimateH;
        else
            %This run did not return a valid homography. Set Estimating to
            %1
            Estimating = 1;
        end
    end
end

end

