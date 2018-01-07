function [ KMatJac, FrameJac ] = SingleImageJacobian( KMatrix, Rotation, ...
        Translation, Correspond, CorrespondIndex)
%SINGLEIMAGEJACOBIAN Function builds, for a single image, the jacobian
%matrix, J, such that the hessian of the error cost function is (J^T)J

%The jacobian will contain partial derivatives of "image point functions"
%with respect to 11 variables: 
% - The five independent components of the KMatrix
% - The six components of the perspectivity

%NOTe: Above, an "image point function" refers to the function which
%transforms an x or y coordinate in the object's frame to a u* or v*
%predicted point on the camera's sensor chip

%The Jacbian will therefore be constructed in two parts, the
%KMatrixJacobian (KMatJac) and the FrameParametersJacobian (FrameJac)

%The jacobian will have a height equal to twice the number of points in the
%consensus set for the image in question (one row relating to each u* or v*
%point).

%Partial derivatives are calculated by computing predicted [u*,v*] points,
%peturbing the parameters one at a time, and then computing them again. The
%difference between the two computations is then divided by the pertubation
%to return first order partial derivatives

%Find the number of points in the consensus set
k = nnz(CorrespondIndex);

%Assign space for the KMatJac and FrameJac
KMatJac = zeros(2*k, 5);
FrameJac = zeros(2*k, 6);

%Compute the predicted [u*,v*] points before any pertubation of the
%parameters

InitialUVPoints = PredictSensorPoints(Correspond, CorrespondIndex, Rotation,...
                            Translation, KMatrix);

%Create a vector containing the initial values of the parameters to be
%optimized
P = zeros(1,11);

%Peturb the parameters one at a time, each time predicting a set of
%[u*,v*] points corresponding to the pertubation. Using this data compute
%the partial derivatives and place them in the Jacobian.

for j = 1:11
    %Reset the parameters at the beginning of each loop so only one
    %parameter is changed at a time. 
    P(1) = KMatrix(1,1);
    P(2) = KMatrix(1,2);
    P(3) = KMatrix(1,3);
    P(4) = KMatrix(2,2);
    P(5) = KMatrix(2,3);
    P(6) = Rotation(1);
    P(7) = Rotation(2);
    P(8) = Rotation(3);
    P(9) = Translation(1);
    P(10) = Translation(2);
    P(11) = Translation(3);

    %Peturb the j'th parameter by a thousandth of itself unless the
    %parameter is the midde top entry in the KMatrix, in which case add a
    %thousandth of itself
    if j ~= 2
        Increment = 0.001*P(j);
    else
        Increment = 0.001;
    end
    
    P(j) = P(j) + Increment;
    
    %Contruct adjusted KMatrix, Rotation and Translation arrays which
    %contain the pertubation 
    [KMatrix_adj, Rotation_adj, Translation_adj] = ...
        BuildAdjustedHomography(P);
    
    %Calculate a new set of [u*,v*] points corresponding to the pertubation
    AdjustedUVPoints = PredictSensorPoints(Correspond, CorrespondIndex, Rotation_adj,...
                            Translation_adj, KMatrix_adj);
    
    %Find the difference between the initial and adjusted points, and
    %divide by the pertubation. This is approximation of the partial
    %derivatives. Place these in the correct location in the jacobian. The
    %derivatives relating to u points are in the top half of the jacobian.
    
    %Assign space for a derivative vector
    Derivative = zeros(1,2*k);
    
    %Find the u related derivatives
    Derivative(1:k) = (AdjustedUVPoints(1,:) - InitialUVPoints(1,:))/Increment;
    
    %Find the v related derivatives
    Derivative(k+1:2*k) = (AdjustedUVPoints(2,:) - InitialUVPoints(2,:))/Increment;
    
    %Place the vector of derivatives in the correct place in the correct
    %jacobian 
    if j <= 5
        KMatJac(:,j) = transpose(Derivative);
    else
        FrameJac(:,j-5) = transpose(Derivative);
    end
    
end

end

