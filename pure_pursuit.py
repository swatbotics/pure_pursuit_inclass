from transform2d import Transform2D
import numpy as np
import matplotlib.pyplot as plt

# T_orig is initial xform from robot to world
# linear_vel and angular_vel are commanded velocities
# dt is timestep
# returns a T_new which is xform from robot to world after timestep dt
def move_robot(T_orig, linear_vel, angular_vel, dt):

    x_new = T_orig.x + linear_vel * np.cos(T_orig.theta) * dt
    y_new = T_orig.y + linear_vel * np.sin(T_orig.theta) * dt
    theta_new = T_orig.theta + angular_vel * dt

    return Transform2D(x_new, y_new, theta_new)



def main():
    
    T_path_from_robot = Transform2D(0, -3.0, 0.25)


    dt = 0.1 # s

    xarr = [ T_path_from_robot.x ]
    yarr = [ T_path_from_robot.y ]

    alpha = 100 # m
    linear_vel = 1.0 # m/s
    ktheta = 1.0 # 1/s

    for i in range(100):

        # implement pure pursuit
        # get pd in the path frame
        xp = T_path_from_robot.x
        yp = T_path_from_robot.y

        pd_path = np.array([ xp + alpha, 0 ])
        
        # get pd in the robot frame
        pd_robot = T_path_from_robot.transform_inv(pd_path)
        
        # steer towards pd
        theta_err = np.arctan2(pd_robot[1], pd_robot[0])

        angular_vel = ktheta * theta_err

        T_path_from_robot = move_robot(T_path_from_robot,
                                       linear_vel, angular_vel, dt)

        xarr.append( T_path_from_robot.x )
        yarr.append( T_path_from_robot.y )
    

    plt.plot(xarr, yarr)
    plt.axis('equal')
    plt.show()
    

if __name__ == '__main__':
    main()

    
