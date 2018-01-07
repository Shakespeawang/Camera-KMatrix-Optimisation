function [ H, BestCorrespondIndex ] = RansacHomog( Correspond, nRuns, MaxError  )
%RANSACHOMOG Function returns an estimate of the normalised homography 
%relating an object's plane and the camera's sensor plane, H, as well as
%an array of the column index (refeering to Correspond) of points within
%the best consensus set, BestCorrespondIndex.

%The function takes as an argument a Correspond matrix containing noisy 
%[u,v] points on the camera's sensor chip and their corresponding [x,y] 
%points in the object's plane.

%The function also takes as arguments nRuns (the number of times the RANSAC 
%code iterates) and MaxError (the maximum admissable error between a point
%predicted by the RANSAC code and the true coordinates of the point).

%Estimation is achieved by encoding simultaneous equations relating [u,v]
%points to their [x,y] counterparts in the following form: 
%Regressor*HomographyVector = DataVector
%Where the Regressor and DataVector both contain information on the
%corresponging points and HomographyVector contains the eight unknown
%elements of the homography (H(3,3)=1). 

%Each pair of corresponding [u,v] and [x,y] points gives a two
%simultaneous equations, meaning four points are required to solve the
%above equation. 

%The RANSAC code runs as follows:
%(1) Randomly choose four points from Correspond and construct Regressor and
%    DataVector. Then invert Regressor to achieve an estimate of H
%(2) Transform all [x,y] points in Correspond through this H to return a set of
%    [u,v] points predited by H. 
%    Compute the error between all predicted [u,v] and the true points in 
%    Correspond. Build a consensus set containing all points for which this error
%    is less than MaxError. 
%(3) Iterate the above steps nRuns number of times
%(4) Take the largest consensus set (containing nBest points) formed from the 
%    iterations and use this to build a larger Regressor matrix of dimensions 
%    (2*nBest)x8. 
%(5) Use a more thorough pseudo-inversion of this larger Regressor to
%    achieve a more accurate estimation of H.

%Check the validity of the inputs
if size(Correspond, 1) ~= 4; 
    error('Correspond has incorrect dimensions')
end

if isinteger(int8(nRuns))== 0 
    error('nRuns must be an integer')
end

if MaxError < 0 
    error('MaxError must be greater than or equal to 0')
end
%END OF CHECKS ON INPUTS

%First define how many points are sensed on the camera's chip
k = length(Correspond);

%Assign space for the estimated homography, H
H = zeros(3,3);

%The code will run nRuns amount of times, choosing different sets of points
%from Correspond, and return the largest consensus set. 

%Define the size of the largest consensus set
nBest = 0;

for Runs = 1:nRuns
    
    %(1)
    %Randomly choose four points, to be used to estimate the homography,
    %until a full rank Regressor matrix can be built 
    RankTest = 1;
    while RankTest == 1
        RankTest = 0;
        %Assign space for the column index of the sample points within
        %Correspond
        SamplePointsIndex = zeros(1,4);
        
        %Choose the index for the first sample point (between 1 and k
        %inclusive). 'Fix' rounds up to the nearest integer. 
        SamplePointsIndex(1,1) = 1 + fix(k*rand);
        if SamplePointsIndex(1,1) > k
            SamplePointsIndex(1,1) = k;
        end
        
        %Choose the index for the following three points randomly,
        %ensuring that there are no repeats
        for j = 2:4
            %Set repeats to zero when a unique index has been chosen
            Repeats = 1;
            while Repeats == 1
                Repeats = 0;
                %Choose a random sample point
                SamplePointsIndex(1,j) = 1 + fix(k*rand);
                if SamplePointsIndex(1,j) > k
                    SamplePointsIndex(1,j) = k;
                end
                %Check that the latest point is not a repeat 
                for p = 1:(j-1)
                    if SamplePointsIndex(1,p) == SamplePointsIndex(1,j)
                        Repeats = 1;
                    end
                end
            end
        end
        
        %Construct the Regressor matrix using the points selected above
        %Assign space for the Resgressor matrix and the data vector
        Regressor = zeros(8,8);
        DataVector = zeros(8,1);
        
        %Build the Regressor matrix two rows at a time. Each pair of rows is
        %built using data in one of the points selected above
        for i = 1:4 
            %Define the row positions of the two rows corresponding to the
            %"i'th" SamplePoint
            r1 = 2*i - 1;
            r2 = 2*i;
            
            %Use the function HomogRowPair to build pairs of rows within
            %Regressor.
            [Regressor(r1:r2, :), DataVector(r1:r2)] = ...
                HomogRowPair(Correspond(:,SamplePointsIndex(1,i)));
        end
        
        %Ensure that the Regressor matrix is full rank (as it should be if
        %four unique points have been chosen). If it is, find out how large
        %the consensus set is for this selection of points (and the 
        %corresponding homography)
        if rank(Regressor) > 7
            %If the Regressor is full rank, calculate the homography based
            %on the four sample points (using a blunt, quick backslash 
            %method).
            HVector = Regressor\DataVector;
            
            %Build the homography matrix from the corresponding elements of
            %HVector
            H(1,1) = HVector(1);
            H(1,2) = HVector(2);
            H(1,3) = HVector(3);
            H(2,1) = HVector(4);
            H(2,2) = HVector(5);
            H(2,3) = HVector(6);
            H(3,1) = HVector(7);
            H(3,2) = HVector(8);
            H(3,3) = 1;
            
            %(2)
            %Keep a count of the number of points in the consensus set for
            %the sample points in consideration
            nCurrent = 0;
            
            %Assign space for an array containing the column index (within
            %Correspond) of all points in the consensus set
            CorrespondIndex = zeros(1,k);
            
            %Go through all points in Correspond and see how large the
            %error is between them and the points predicted by the current
            %homography. If the error is less than the maximum admissable
            %error, record the points column index in ConsensusIndex
            for j = 1:k
                %Define a homogeneous point (in the object's frame) related 
                %to the specified location in Correspond
                HomogeneousPoint_object = [Correspond(3,j); Correspond(4,j)...
                    ;1];
                %Use the estimated homography to transform the homogeneous 
                %point to one in the camera's frame
                HomogeneousPoint_camera = H*HomogeneousPoint_object;
                
                %Scale the homogeneous point by the homogeneous scaler to
                %recover the [u,v] coordinates (in the camera chip's frame)
                %predicted by the homography
                
                %Assign space for the point predicted by the homography
                PredictedPoint = zeros(2,1);
                PredictedPoint(1) = ... 
                    HomogeneousPoint_camera(1)/HomogeneousPoint_camera(3);
                PredictedPoint(2) = ... 
                    HomogeneousPoint_camera(2)/HomogeneousPoint_camera(3);
                
                %Check the error between this point and the true
                %('measured') point stored in Correspond
                
                %Define the true [u,v] point stored in Correspond
                TruePoint = [Correspond(1,j); Correspond(2,j)];
                ThisError = norm(PredictedPoint - TruePoint);
                
                %Check if the error is less than the maximum acceptable
                %error. If so, store it's column index in CorrespondIndex
                %and increment nCurrent (the number of points in this
                %consensus set)
                if ThisError < MaxError
                    nCurrent = nCurrent + 1;
                    CorrespondIndex(nCurrent) = j;
                end
            end
            
            %See if these set of sample points (and their corresponding
            %homography) did better than all previous attempts
            if nCurrent > nBest 
                nBest = nCurrent;
                BestCorrespondIndex = CorrespondIndex;
            end
        else
            %(For clarity) The code carries out this 'else' if the Regressor
            %matrix is not full rank. It sets RankTest to 1, prompting the
            %code to select another four sample points from Correspond
            RankTest=1;
        end
    end
    %(3) 
    %Iterate 
end

%(4)
%BestCorrespondIndex now contains the locations of all points, within
%Correspond, of the points within the largest consensus set
%from the test above.
%Using the points within the this consensus set, use a robust pseudo-inverse
%method to re-estimate the homography
if nBest > 0 
    %The Regressor matrix and DataVector must now be large enough to
    %accommodate all points in the largest consensus set
    Regressor = zeros(2*nBest, 8);
    DataVector = zeros(2*nBest, 1);

    %Build Regressor and DataVector two rows at a time with HomogRowPair,
    %as above
    for i = 1:nBest
        r1 = 2*i - 1;
        r2 = 2*i;

        [Regressor(r1:r2, :), DataVector(r1:r2)] = ...
            HomogRowPair(Correspond(:,BestCorrespondIndex(1,i)));
    end
    
    %The pseudo-inverse method derives from singular value
    %decomoposition. Compute the SVD of the Regressor
    [U,D,V] = svd(Regressor, 'econ');

    %Analyse how well conditioned the Regressor is (how reliably the 
    %pseudo-inverse can be found) and set the 'condition number'
    if D(8,8) < eps
        %This is a poorly conditioned system of equations, set the
        %condition number arbitrarily large
        ConditionNumber = 1e16;
    else
        ConditionNumber = D(1,1)/D(8,8);
    end

    if ConditionNumber > 1e8
        %If the system is too poorly conditioned, return a zero
        %homography
        H = zeros(3,3);
        fprintf('Homography cannot be reliably estimated \n')
        fprintf('Regressor matrix is poorly conditioned \n')
    else 
        %If the conditioning is good, compute the pseudo-inverse
        %The pseudo-inverse can be found to be (V)*(D^-1)*(U^T)

        %Invert D. D is diagonal so finding its inverse is equal to 
        %replacing the singular values on the leading diagonal with
        %their reciprocal

        %Assign space for the inverse
        invD = zeros(8,8);

        for i = 1:8
            invD(i,i) = 1/D(i,i);
        end

        %The pseudo inverse:
        invRegressor = V*(invD*transpose(U));

        %Compute the vector HVector containing the elements of the
        %homography
        HVector = invRegressor*DataVector;

        %Build the homography matrix from the corresponding elements of
        %HVector
        H(1,1) = HVector(1);
        H(1,2) = HVector(2);
        H(1,3) = HVector(3);
        H(2,1) = HVector(4);
        H(2,2) = HVector(5);
        H(2,3) = HVector(6);
        H(3,1) = HVector(7);
        H(3,2) = HVector(8);
        H(3,3) = 1;
    end
else 
    %Indicates that no suitable consensus sets could obtained from any
    %of the runs of the RANSAC code. In this case return a homography
    %of zeros
    H = zeros(3,3);
    fprintf('Homography cannot be reliably estimated \n')
    fprintf('RANSAC did not return a non-zero consensus set \n')
end     

end

