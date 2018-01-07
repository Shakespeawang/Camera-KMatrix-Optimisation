function [ R ] = RandomRotationMatrix( )
%RANDOMROTATIONMATRIX Generates a random rotation matrix, R, by generating
%three orthogonal unit vectors and placing them inside a 3x3 matrix

%Generate an initial random unit vector, x_hat, by normalising a random vector, x 

%x must have a length greater than 'eps' for computational reliability

xnorm = 0;

while xnorm < eps
    x = rand(3,1);
    xnorm = norm(x);
end 

x_hat = x/xnorm;

%Next generate a second random vector unit, p

%Find the (magnitude of the) component of p which is parallel to x_hat and deduct it from p
%in the direction of x_hat, thereby leaving a vector perpendicular to x_hat

%Again, we ensure that the resultant vector, p_perp, is not too small

p_perp_norm = 0;

while p_perp_norm < eps
    
    p = rand(3,1);
    p_pll = dot(x_hat, p);               %The magnitude of the component of p parallel to x_hat
    p_perp = p - p_pll*x_hat;            %The component of p perpendicular to x_hat   
    p_perp_norm = norm(p_perp);
    
end

%Normalise p_perp, this is the second vector of the orthogonal set, y_hat

y_hat = p_perp/p_perp_norm;

%The third vector in the set is the cross product of x_hat and y_hat

z_hat = cross(x_hat, y_hat);

%Build the rotation matrix
R = [x_hat, y_hat, z_hat];

end

