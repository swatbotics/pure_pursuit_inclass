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
    
    T_world_from_robot = Transform2D(0, 0, 0)

    linear_vel = 0.3 # m/s
    angular_vel = 0.5 # rad/s
    
    dt = 0.1 # s

    xarr = [ T_world_from_robot.x ]
    yarr = [ T_world_from_robot.y ]

    for i in range(100):
        
        T_world_from_robot = move_robot(T_world_from_robot,
                                        linear_vel, angular_vel, dt)
        
        xarr.append( T_world_from_robot.x )
        yarr.append( T_world_from_robot.y )

    plt.plot(xarr, yarr)
    plt.axis('equal')
    plt.show()

if __name__ == '__main__':
    main()

    
