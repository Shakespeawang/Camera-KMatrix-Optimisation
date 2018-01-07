%Script finds an estimate for the KMatrix of a camera

%NOTE: Random camera initialisation can result in the Cholesky product not
%being positive definite. This script may need to be run several times
%before the Cholesky factorisation runs well

clear

tic;

%Define the parameters of the camera to be modelled
Parameters = [3200;           %Width of the camera chip (x-pixels, integer)      
              4000;           %Height of the camera chip (y-pixels, integer)
              35;             %Focal length of the lens (mm)  
              0.0012;         %Effective width of a pixel in the x direction (mm)  
              0.0012;         %Effective height of a pixel in the y direction (mm)  
              0.01;           %Skewness of the pixels (non-dimensional)  
              0.5;            %Principal direction offset in x (fraction of chip width)  
              0.5];           %Principal direction offset in y (fraction of chip height)  
                    
%Build a structure containing the KMatrix, chip height and chip width of
%the camera. 
          
CameraStructure = BuildCamera(Parameters);

%Contruct a square, tiled grid with side length 'GridWidth' and tile size
%of 'GridIcrement'. Function returns homogeneous Points of the grid in
%its own coordinate system

%Lengths in mm
GridWidth = 1000;
GridIncrement = 10;
GridPoints = BuildGrid(GridWidth, GridIncrement);

%Position the grid randomly in world coordinates, with random orientation.
%Do this by a homogeneous transformation T_ow, defining the transformation
%from object coordinates to world coordinates. 
%The x-y plane of this coordinate system is the plane to be acted on by the
%homography
T_ow = PositionObject();

%Scale points in the grid and on the camera's chip to improve conditioning
%of the homography estimation and reduce computational requirment. These
%scalings ensure that all points on the camera and grid lie within -1 and 1
%in both dimensions.
%The scalings are stored within a structure 'Scale'

Scale.GridScale = 2/GridWidth;
if CameraStructure.ChipWidth >= CameraStructure.ChipHeight
    Scale.CameraScale = 2/CameraStructure.ChipWidth;
else
    Scale.CameraScale = 2/CameraStructure.ChipHeight;
end

%% Compute the KMatrix

%Now generate homographies for 'nImages' different views of the grid.
%nImages must be >= 3 to generate a full rank Regressor for the KMatrix
%estimation

nImages = 6;

HomogData = GenerateHomogData(nImages, T_ow, GridPoints, CameraStructure...
    , Scale, GridWidth);

%HomogData now contains 'nImages' homographies and their asscociated data

%Construct the Regressor matrix, which will be used to estimate
%transpose((K^-1))(K^-1)) (the Cholesky product), two rows at a time. Each
%pair of rows is built using data from one of the homographies in HomogData

%Assign space for the regressor
Regressor = zeros(2*nImages, 6);

for n=1:nImages
    
    %Define the row index for the two rows being built on this iteration
    r1 = 2*n - 1;
    r2 = 2*n;
    
    %Pass the homographies into KMatrixRowPair one at a time
    Regressor(r1:r2, :) = KMatrixRowPair(HomogData{n,1});
end

%The vector containing the elements of the Cholesky product is the kernel
%of the Regressor matrix. Noise will preclude an exact solution, so find an
%estimate of the kernel:

%Calculate the singular value decomoposition of the regressor
[U,D,V] = svd(Regressor, 'econ');
%Create a vector of the singular values
X = diag(D);
%Find the smallest singular value and it's location within D
[m, i] = min(X);
%The estimate of the kernel, PhiVector, is the singular right hand vector
%corresponding to to the smallest singular value
PhiVector = V(:,i);

%Ensure that the estimate is positive definite
if PhiVector(1) < 0
    PhiVector = -PhiVector;
end

%Construct the Cholesky Product matrix, P, from the elements of PhiVector
P(1,1) = PhiVector(1);
P(1,2) = PhiVector(2);
P(1,3) = PhiVector(3);
P(2,2) = PhiVector(4);
P(2,3) = PhiVector(5);
P(3,3) = PhiVector(6);

%The Cholesky Product is symmentric, fill in the remaining components
%accordingly

P(2,1) = P(1,2);
P(3,1) = P(1,3);
P(3,2) = P(2,3);

%Make sure P is positive definite by checking if all its eigenvalues are
%positive
if any( eig(P) < -eps)
    error('Cholesky Product is not positive definite')
end

%Use the Cholesky factorization of P to recover the inverse of the KMatrix
invK = chol(P);

%Find the KMatrix estimate
KMatrixEstimate = invK \ eye(3);

%The scaling of the points on the camera's sensor chip has affected the
%KMatrix. Reverse the effect of this scaling 

%Normalise the KMatrix
KMatrixEstimate = KMatrixEstimate/KMatrixEstimate(3,3);

%% Now optimise the KMatrix

%Start by initialising the KMatrix, setting it to the initial estimate
KMatrix = KMatrixEstimate;

%Check the validity of the KMatrix 
%Check KMatrix is upper diagonal
if CameraStructure.KMatrix(2,1) ~=0 || CameraStructure.KMatrix(3,1) ~=0 ...
        || CameraStructure.KMatrix(3,2) ~=0
    error('KMatrix is not upper diagonal')
end

%Check that the bottom right entry of the KMatrix is 1, if not, normalise
%it
if KMatrix(3,3) ~=1
    KMatrix = KMatrix/KMatrix(3,3);
end

%Define indices to access the cell array containing the homography data for
%clarity later in the code
NHOMOGRAPHY = 1;
NCORRESPOND  = 2;
NINDEX = 3;

%Extract how many images were generated for the estimation
nImages = size(HomogData, 1);

%Generate more realistic estimates of the perspectivities in each of the
%'nImages' homographies generated. The perspectivities are initially of the
%form [rX rY t], and will be stored in a Shifted-Angle Axis representation.
%Store these more realistic perspectivities in a data cell.
FrameParameters = cell(nImages, 2);
%Assign labels for accessing the cell
NROTATION = 1;
NTRANSLATION = 2;

%Assign space for the rotation matrix to be generated from the KMatrix seed
RotationMatrix = zeros(3,3);

%Find the perspectivities for each of the images
for k = 1:nImages
    
    %Extract the initial perspectivity from the homography referring to the
    %image in question
    Homography = HomogData{k, NHOMOGRAPHY};
    Perspectivity = KMatrix \ Homography;
    
    %Scale the rX component of the perspectivity to ensure its a unit
    %vector. Scale the rest of the perspectivity by rX as well.
    
    %Ensure the normalisation can be computed reliably
    rX = Perspectivity(:, 1);
    if norm(rX) < eps
        error('Invalid homography generated \n Perspectivity cannot be normalised reliably')
    end
    
    Perspectivity = Perspectivity / norm(rX) ;
    rX = Perspectivity(:,1);
    
    %Extract the translation component, t, from the perspectivity
    t = Perspectivity(:, 3);
    
    %Build a valid rotation matrix with reference to rX. A similar process
    %has been carries out earlier in the project.
    rY = Perspectivity(:, 2);
    
    rY_pll = dot(rX, rY);           %The magnitude of the component of rY parallel to rX
    rY_perp = rY - rY_pll*rX;       %The component of rY perpendicular to rX, this will be 
                                    %the direction of the adjusted rY vector 
    
    %Ensure that the adjusted rY vector can be normalised reliably
    if norm(rY_perp) < eps
        error('Invalid homography generated \n Perspectivity cannot be normalised reliably')
    end
    
    rY = rY_perp/norm(rY_perp);
    
    %Find the final vector in the rotation matrix as the cross product of
    %rX and rY 
    rZ = cross(rX, rY);
    
    %Construct the adjusted (more realistic) rotation matrix for this image
    RotationMatrix = [rX rY rZ];
        
    %Now convert this rotation to Shifted-Angle Axis form
    %Find the angle of rotation and shift it by 4pi to avoid allow negative
    %angles and remove the effect of discontinuities around angle=0. (Any
    %shift that is a multiple of 2pi will do)
    CosAngle = (trace(RotationMatrix) - 1)/2;
    
    %Ensure that |CosAngle| =< 1
    if CosAngle > 1
        CosAngle = 1;
    end
    if CosAngle < -1
        CosAngle = -1;
    end
    
    RotationAngle = acos(CosAngle);
    RotationAngle = RotationAngle + 4*pi;
    
    %Find the axis of rotation by finding the eigenvector corresponding to
    %the real eigenvalue (the real eigenvalue should = 1 but may due to
    %rounding errors)
    [V,D] = eig(RotationMatrix);
    %Convert the diagonal matrix of eigenvalues to a vector
    D = diag(D);
    %Find the eigenvalue closest to unity 
    Difference = 1 - D;
    [M, i] = min(real(Difference));
    M = D(i);
    %Find the eigenvector corrseponding to this eigenvalue
    RotationAxis = V(:, i);
    %Normalise it
    if norm(RotationAxis) < eps
        error('Invalid perspectiviy \n RotationAxis cannot be normalised reliably')
    end 
    RotationAxis = RotationAxis/norm(RotationAxis);
    
    %It is possible that the rotation axis is in the wrong direction. Check
    %if this is the case and, if so, invert the axis
    RPLUS = RodriguesRotation(RotationAxis, RotationAngle);
    RMINUS = RodriguesRotation(-RotationAxis, RotationAngle);
    
    %Check which of the above two rotation matrices is closest to the
    %original
    
    if norm(RotationMatrix - RPLUS) > norm(RotationMatrix - RMINUS)
        RotationAxis = -RotationAxis;
    end
    
    %Generate the Shifted-Angle representation 
    RotationAxis = RotationAxis*RotationAngle;
    
    %Store this data (encoding the position of the grid for this
    %homography) in FrameParameters
    FrameParameters{k, NROTATION} = RotationAxis;
    FrameParameters{k, NTRANSLATION} = t;
    
end

%Now go through the Levenberg-Marquadt steps to optimise the KMatrix.
%Assign space for a cell array containing data on the optimisation. 
%The data includes: 
%1) The Error Vector
%2) The KMatrix Jacobian
%3) FrameParametersJacobian

%For clarity, define the cell array indices corresponding to the above
%parameters
NERROR = 1;
NKMATRIXJACOBIAN = 2; 
NFRAMEPARAMETERSJACOBIAN = 3;

OptComponents = cell(nImages, 3);

%Allocate space for the Hessian approximation: (J^T)J
%It has dimensions (5+6*nImages)x(5+6*nImages), with a row and column for
%each of the parameters to be changed
ProblemSize = 5+6*nImages;
JTJ = zeros(ProblemSize);

%Initialise the current error
CurrentError = 0;

%Assign space for the gradient, dE/dp, which has length equal to the number
%of parameters
Gradient = zeros(ProblemSize, 1);

%Compute the vectors and matrices required for the optimisation, to be
%stored in OptComponents
for j=1:nImages
    %Compute the error vector for this image, it will have a length equal
    %to two times the number of points in the consensus set for the image
    OptComponents{j, NERROR} = BuildErrorVector(KMatrix, FrameParameters{j,...
        NROTATION}, FrameParameters{j,NTRANSLATION}, HomogData{j,NCORRESPOND}...
        ,HomogData{j,NINDEX});
    
    %Compute the magnitude of the cost function 
    CurrentError = CurrentError + 0.5*OptComponents{j,NERROR}'...
        *OptComponents{j,NERROR};
   
    %Build a Jacobian for a single image, and piece them together later
    [OptComponents{j, NKMATRIXJACOBIAN}, OptComponents{j,NFRAMEPARAMETERSJACOBIAN}]...
        = SingleImageJacobian(KMatrix, FrameParameters{j, NROTATION}, ...
        FrameParameters{j,NTRANSLATION}, HomogData{j, NCORRESPOND},... 
        HomogData{j, NINDEX});
    
    %Place the jacobian generated above (for a single image) in the larger
    %Hessian apporixamtion (J^T*J)
    
    %Begin by filling in the top left 5x5 block. Which is the sum of the
    %inner products of all 'KMatrixJacobians'.
    JTJ(1:5,1:5) = JTJ(1:5,1:5) + ...
        OptComponents{j,NKMATRIXJACOBIAN}'*OptComponents{j,NKMATRIXJACOBIAN};
    
    %Fill in the diagonal block, which is the inner product of the
    %FrameParametersJacobian computed above
    %Define start and end rows that change for each image
    Start = 6 +(j-1)*6;
    End = Start + 5;
    
    JTJ(Start:End, Start:End) = ...
    OptComponents{j,NFRAMEPARAMETERSJACOBIAN}'... 
        *OptComponents{j,NFRAMEPARAMETERSJACOBIAN};
    
    %Fill in the first row block of the hessian, which is the product of
    %the KMatrixJacobian and FrameParametersJacobian 
    JTJ(1:5, Start:End) = ...
        OptComponents{j,NKMATRIXJACOBIAN}'*...
        OptComponents{j,NFRAMEPARAMETERSJACOBIAN};
    
    %Reflect the matrix about the leading diagonal to make it symmetric
    JTJ(Start:End, 1:5) = JTJ(1:5, Start:End)';
    
    %Calculate the correponding section of the gradient vector
    Gradient(1:5) = Gradient(1:5) + ...
        OptComponents{j,NKMATRIXJACOBIAN}'*OptComponents{j,NERROR};
    Gradient(Start:End) = ... 
        OptComponents{j,NFRAMEPARAMETERSJACOBIAN}'*OptComponents{j,NERROR};
    
end

%We now have a value for the cost function, a gradient vector and a hessian
%matrix. We are now in a position to optimise the model.

%Store the initial error for later analysis
InitialError = CurrentError;

%Initialise mu, the damping variable of the L-M method
mu = max(diag(JTJ))*0.01;

%Initialise nu, the variable used to control the growth of mu
nu = 2;

%Set a toggle to be set to zero once the optimisation is complete
Searching = 1;

%Initialise a variable to count the iterations of the optimisation code
Iterations = 0;

%Define the maximum number of iterations
MaxIterations = 100;

%Initialise a variable to extract the total change in parameter values
dpTotal = 0;

%Carry out the optimisation
while Searching == 1
    
    Iterations = Iterations + 1;
    
    %Exit the search loop if too many iterations have occurred
    if Iterations > MaxIterations
        fprintf('More iterations than MaxIterations')
        break 
    end
    
    %Check if the problem has converged 
    if norm(Gradient)/ProblemSize < 0.0001
        fprintf('Cost function mimised')
        break 
    end
    
    %Solve the L-M equation for the change in paramter vector
    dp = -(JTJ + mu*eye(ProblemSize))\Gradient;
    dpTotal = dpTotal + dp;
    
    %Calculate the predicted change in cost function (as predicted by the
    %current jacobian)
    PredictedChange = Gradient'*dp;
    
    %Initialise variables which will contain the petrubed parameters. Set
    %them equal to the parameters as they were before the optimisation.
    %Later, if this peturbation causes the cost function to reduce, we will
    %overwrite the input parameters with the updated parameters, thereby
    %causing the peturbation variables to take the more optimal values.
    PeturbedKMatrix = KMatrix;
    PeturbedFrameParameters = FrameParameters;
    
    %Peturb the KMatrix parameters by the values in dp
    PeturbedKMatrix(1,1) = PeturbedKMatrix(1,1) + dp(1);
    PeturbedKMatrix(1,2) = PeturbedKMatrix(1,2) + dp(2);
    PeturbedKMatrix(1,3) = PeturbedKMatrix(1,3) + dp(3);
    PeturbedKMatrix(2,2) = PeturbedKMatrix(2,2) + dp(4);
    PeturbedKMatrix(2,3) = PeturbedKMatrix(2,3) + dp(5);
    
    %Initialise the new error for this peturbation in parameters
    NewError = 0;
    
    %Peturb the parameters corresponding to the perpectivity of each of the
    %images
    for j=1:nImages
        
        %Define the a start location which changes with the image 
        Start = 6 + (j-1)*6;
        PeturbedFrameParameters{j,NROTATION} =...
            PeturbedFrameParameters{j,NROTATION} + dp(Start:Start+2);
        
        PeturbedFrameParameters{j,NTRANSLATION} =...
            PeturbedFrameParameters{j,NTRANSLATION} + dp(Start+3:Start+5);
        
        %Compute the error for this image
        OptComponents{j,NERROR} = BuildErrorVector(PeturbedKMatrix,...
            PeturbedFrameParameters{j,NROTATION},...
            PeturbedFrameParameters{j,NTRANSLATION},...
            HomogData{j,NCORRESPOND}, HomogData{j,NINDEX});
        
        %Compute the error for this image and add it to the error for all
        %previous images
        NewError = NewError + 0.5*...
            OptComponents{j,NERROR}'*OptComponents{j,NERROR};
        
    end
    
    %NewError now has a value equal to the cost function for this iteration
    
    %Calculate how much the error has changed between iterations
    ChangeInError = NewError - CurrentError;
    
    %Calculate the gain, rho, to see how good the current jacobian is at
    %predicting the change in error
    rho = ChangeInError/PredictedChange;
    
    %See if the error has gone up or down
    if ChangeInError > 0 
        %Error has gone up, increase mu
        mu = mu*nu;
        %Adjust nu as well
        nu = 2*nu;
    else
        %The error has gone down, adjust mu according to Madsen, Nielsen
        %and Tingleff
        mu = mu*max([1/3, (1-(2*rho -1)^3)]);
        %The error has gone down, reset nu to its original value
        nu = 2;
        
        %Accept the peturbed parameters
        KMatrix = PeturbedKMatrix;
        FrameParameters = PeturbedFrameParameters;
        
        %Update the error
        CurrentError = NewError;
        
        %If the gain, rho, is less than a critical value, it indicates the
        %current jacobian is inaccurate. Due to the expense of calculating
        %it, only recalculate the jacobian in this case. 
        if rho < 1/3
            JTJ = zeros(ProblemSize);
            for j=1:nImages
                %The jacobian for one image
                [OptComponents{j, NKMATRIXJACOBIAN},...
                    OptComponents{j,NFRAMEPARAMETERSJACOBIAN}]...
                    = SingleImageJacobian(KMatrix, FrameParameters{j, NROTATION}, ...
                    FrameParameters{j,NTRANSLATION}, HomogData{j, NCORRESPOND},...
                    HomogData{j, NINDEX});
                
                %Contruct JTJ as above
                Start = 6 +6*(j-1);
                End = Start + 5;
                
                JTJ(Start:End, Start:End) = ...
                    OptComponents{j,NFRAMEPARAMETERSJACOBIAN}'...
                    *OptComponents{j,NFRAMEPARAMETERSJACOBIAN};
                
                %Fill in the first row block of the hessian, which is the
                %product of the KMatrixJacobian and FrameParametersJacobian
                JTJ(1:5, Start:End) = ...
                    OptComponents{j,NKMATRIXJACOBIAN}'*...
                    OptComponents{j,NFRAMEPARAMETERSJACOBIAN};
                
                %Reflect the matrix about the leading diagonal to make it
                %symmetric
                JTJ(Start:End, 1:5) = JTJ(1:5, Start:End)';
            end
        end
        
        %Compute the gradient for this iteration
        Gradient = zeros(ProblemSize,1);
        for j = 1:nImages
            Start = 6 +6*(j-1);
            End = Start + 5; 
            Gradient(1:5) = Gradient(1:5) + ...
                OptComponents{j,NKMATRIXJACOBIAN}'*OptComponents{j,NERROR};
            Gradient(Start:End) = ...
            OptComponents{j,NFRAMEPARAMETERSJACOBIAN}'*OptComponents{j,NERROR};
        end
    end
end

%% Now store the current estimate as the optimised KMatrix
KMatrixOptimised = KMatrix;

%Reverse the translation
KMatrixOptimised(1:2, 3) = KMatrixOptimised(1:2, 3) + [1;1];

%Reverse the scaling
KMatrixOptimised(1:2,1:3) = KMatrixOptimised(1:2,1:3)/Scale.CameraScale;

%Do the same for the seed
KMatrixEstimate(1:2, 3) = KMatrixEstimate(1:2, 3) + [1;1]; 
KMatrixEstimate(1:2,1:3) = KMatrixEstimate(1:2,1:3)/Scale.CameraScale;


%Print for analysis: Seed estimate, optimised estimate, actual KMatrix
%Also print the change in error in optimisation
OverallErrorChange = InitialError - CurrentError
ErrorReduction = OverallErrorChange/InitialError
Seed = KMatrixEstimate
KMatrixOptimised
CameraStructure.KMatrix

%Find the element-wise percentage error between the seed estimate and true
%KMatrix
ErrorSeed = AbsoluteDifference(CameraStructure.KMatrix, Seed);
SeedAverage = mean([ErrorSeed(1,1), ErrorSeed(2,2), ...
                    ErrorSeed(1,3), ErrorSeed(2,3)])
                
%Find the element-wise percentage error between the seed estimate and true
%KMatrix
ErrorOptimised = AbsoluteDifference(CameraStructure.KMatrix, KMatrixOptimised);
OptimisedAverage = mean([ErrorOptimised(1,1), ErrorOptimised(2,2), ...
                    ErrorOptimised(1,3),ErrorOptimised(2,3)])

toc;