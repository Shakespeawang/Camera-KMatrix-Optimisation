function [ T_cw ] = PositionCamera2( T_ow, CameraDistance )
%POSITIONCAMERA2 Function places the camera at a distance CameraDistance
%away from an object positioned and oriented according to T_ow. The camera
%view is oriented almost orthogonally to the x-y plane of the object, with
%a random pertubation. The function returns a 4x4 matrix T_cw which defines 
%a homogeneous transformation from camera to world coordinates

%The orientation of the view vector between camera and object is generated
%by rotating a vector orthogonal to the x-y plane about a randomly generated
%axis in the plane 

%Check the integrity of the inputs
if CameraDistance<= 0 
    error('Viewing distance of camera must be positive')
end

TestTransformMatrix(T_ow, 'T_ow')

%END OF INPUT CHECKS

%Assign space for T_cw
T_cw = zeros(4,4);

%Extract the z-axis of the object coordinate system from T_ow. The camera
%will view the object from near the negative z-axis direction
neg_object_zaxis = -T_ow(1:3, 3);

%Generate a random unit vector in the x-y plane of the object coordinate 
%system

%Ensure the vector generated can be reliably normalised
xy_plane_vector_norm = 0;

while xy_plane_vector_norm < eps;
    
    object_xaxis = T_ow(1:3, 1);    %x-axis of the object coordinate system
    object_yaxis = T_ow(1:3, 2);    %y-axis of the object coordinate system
    
    %Generate a random vector in the plane by a random combination of the
    %above two vectors
    xy_plane_vector = rand*object_xaxis + rand*object_yaxis;  
    
    xy_plane_vector_norm = norm(xy_plane_vector);
end

%Normalise the vector in the x-y plane
xy_plane_vector = xy_plane_vector/xy_plane_vector_norm;

%Generate a random angle between 0 and 15 degrees (0 and pi/12 radians)
random_angle = rand*(pi/12);

%Use RodriguesRotation to rotate the negative z-axis about xy_plane_vector by
%random_angle. Define this vector as the z-axis of the camera
T = RodriguesRotation(xy_plane_vector, random_angle);
CameraZ = T*neg_object_zaxis;

%Define an x-axis of the camera that is perpendicular to the z axis - by
%finding the component of a random vector, p, which is perpendicular to
%CameraZ (the same method used in RandomRotationMatrix).

p_perp_norm = 0;

while p_perp_norm < eps
    
    p = rand(3,1);
    p_pll = dot(CameraZ, p);       %The magnitude of the component of p parallel to CameraZ
    p_perp = p - p_pll*CameraZ;    %The component of p perpendicular to CameraZ   
    p_perp_norm = norm(p_perp);
    
end

%The x-axis is the normalised p_perp
CameraX = p_perp/p_perp_norm;

%The y-axis the cross product of CameraZ and CameraX
CameraY = cross(CameraZ, CameraX);

%Finally, with the orientation of the camera established, determine the
%camera's position. 

%Define the vector from which the camera views the object 
ViewVector = CameraDistance*CameraZ;

%Extract the position of the object from T_ow
ObjectPosition = T_ow(1:3, 4);

%Define the camera position such that it views the origin of the object
%coordinate system via ViewVector
CameraPosition = ObjectPosition - ViewVector;

%Finally assemble T_cw 
T_cw(1:3, 1:4) = [CameraX, CameraY, CameraZ, CameraPosition];

%Set the T_cw(4,4) to 1 to complete T_cw
T_cw(4,4) = 1;

end
