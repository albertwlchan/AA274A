import numpy as np

EPSILON_OMEGA = 1e-3

def compute_dynamics(xvec, u, dt, compute_jacobians=True):
    """
    Compute Turtlebot dynamics (unicycle model).

    Inputs:
                     xvec: np.array[3,] - Turtlebot state (x, y, theta).
                        u: np.array[2,] - Turtlebot controls (V, omega).
        compute_jacobians: bool         - compute Jacobians Gx, Gu if true.
    Outputs:
         g: np.array[3,]  - New state after applying u for dt seconds.
        Gx: np.array[3,3] - Jacobian of g with respect to xvec.
        Gu: np.array[3,2] - Jacobian of g with respect to u.
    """
    ########## Code starts here ##########
    # TODO: Compute g, Gx, Gu
    # HINT: To compute the new state g, you will need to integrate the dynamics of x, y, theta
    # HINT: Since theta is changing with time, try integrating x, y wrt d(theta) instead of dt by introducing om
    # HINT: When abs(om) < EPSILON_OMEGA, assume that the theta stays approximately constant ONLY for calculating the next x, y
    #       New theta should not be equal to theta. Jacobian with respect to om is not 0.
    x, y, th = xvec
    V, om = u
    th_new = th + om * dt
    if abs(om) < EPSILON_OMEGA:
    	g = np.array([x + V * np.cos(th) * dt, y + V * np.sin(th) * dt, th_new])
    	
    	Gx = np.array([[1, 0, -V * np.sin(th) * dt],
    		      [0, 1,  V * np.cos(th) * dt],
    		      [0, 0, 1]])
    	Gu = np.array([[np.cos(th) * dt, -V * dt ** 2 * np.sin(th)/2],
    		       [np.sin(th) * dt, V * dt ** 2 * np.cos(th)/2],
    		       [0, dt]])
	
    else:
    	g = np.array([x + V / om * (np.sin(th_new) - np.sin(th)), y + V / om * (np.cos(th) - np.cos(th_new)), th + om * dt])
    	Gx = np.array([[1, 0, V / om * (np.cos(th_new) - np.cos(th))],
    		       [0, 1, V / om * (np.sin(th_new) - np.sin(th))],
    		       [0, 0, 1]])
    	du1 = V / om * dt * np.cos(th_new) - V / (om ** 2) * (np.sin(th_new) - np.sin(th)) 
    	du2 = V / om * dt * np.sin(th_new) - V / (om ** 2) * (np.cos(th) - np.cos(th_new))
    	Gu = np.array([[1 / om * (np.sin(th_new) - np.sin(th)), du1],
		       [1 / om * (np.cos(th) -np.cos(th_new)), du2],
		       [0, dt]])



    ########## Code ends here ##########

    if not compute_jacobians:
        return g

    return g, Gx, Gu

def transform_line_to_scanner_frame(line, x, tf_base_to_camera, compute_jacobian=True):
    """
    Given a single map line in the world frame, outputs the line parameters
    in the scanner frame so it can be associated with the lines extracted
    from the scanner measurements.

    Input:
                     line: np.array[2,] - map line (alpha, r) in world frame.
                        x: np.array[3,] - pose of base (x, y, theta) in world frame.
        tf_base_to_camera: np.array[3,] - pose of camera (x, y, theta) in base frame.
         compute_jacobian: bool         - compute Jacobian Hx if true.
    Outputs:
         h: np.array[2,]  - line parameters in the scanner (camera) frame.
        Hx: np.array[2,3] - Jacobian of h with respect to x.
    """
    alpha, r = line

    ########## Code starts here ##########
    # TODO: Compute h, Hx
    # HINT: Calculate the pose of the camera in the world frame (x_cam, y_cam, th_cam), a rotation matrix may be useful.
    # HINT: To compute line parameters in the camera frame h = (alpha_in_cam, r_in_cam), 
    #       draw a diagram with a line parameterized by (alpha,r) in the world frame and 
    #       a camera frame with origin at x_cam, y_cam rotated by th_cam wrt to the world frame
    # HINT: What is the projection of the camera location (x_cam, y_cam) on the line r? 
    # HINT: To find Hx, write h in terms of the pose of the base in world frame (x_base, y_base, th_base)
    x_base, y_base, th_base = x
    
    x_cam_base, y_cam_base, th_cam_base = tf_base_to_camera
     
    T1 = np.array([[np.cos(th_base), -np.sin(th_base), x_base],  #transformation from origin to robot base
    		      [np.sin(th_base), np.cos(th_base), y_base],
    		      [0, 0, 1]])

    T = T1 @ np.array([x_cam_base,y_cam_base,1]) 
    camera_world = T[0:2] 
    x_cam_w = camera_world[0]
    y_cam_w = camera_world[1]
    
    th_cam_world = th_base + th_cam_base

    h = np.array([alpha - th_cam_world ,r - x_cam_w*np.cos(alpha) - y_cam_w*np.sin(alpha)])

    Hx = np.array([[ 0 , 0 , -1], [-np.cos(alpha), -np.sin(alpha), (y_cam_base*np.cos(alpha)-x_cam_base*np.sin(alpha))*np.cos(th_base) + (x_cam_base*np.cos(alpha)+y_cam_base*np.sin(alpha))*np.sin(th_base)]])
        
    ########## Code ends here ##########
    
    if not compute_jacobian:
        return h

    return h, Hx


def normalize_line_parameters(h, Hx=None):
    """
    Ensures that r is positive and alpha is in the range [-pi, pi].

    Inputs:
         h: np.array[2,]  - line parameters (alpha, r).
        Hx: np.array[2,n] - Jacobian of line parameters with respect to x.
    Outputs:
         h: np.array[2,]  - normalized parameters.
        Hx: np.array[2,n] - Jacobian of normalized line parameters. Edited in place.
    """
    alpha, r = h
    if r < 0:
        alpha += np.pi
        r *= -1
        if Hx is not None:
            Hx[1,:] *= -1
    alpha = (alpha + np.pi) % (2*np.pi) - np.pi
    h = np.array([alpha, r])

    if Hx is not None:
        return h, Hx
    return h

