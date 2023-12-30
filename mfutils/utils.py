import numpy as np
import matplotlib.pyplot as plt

# Utility functions
def rot2d(angle):
    return np.array([[np.cos(angle), -np.sin(angle)],
                     [np.sin(angle), np.cos(angle)]])

# Plotting functions
def plot_lidar_coords(lidar_coords_x, lidar_coords_y, marker='o', color='r'):
    style = marker + color
    return(plt.plot(lidar_coords_x, lidar_coords_y, style))

def plot_arrow(x, y, yaw, arrow_length=1.0,
               origin_point_plot_style="xr",
               head_width=0.1, fc="r", ec="k", **kwargs):
    """
    Plot an arrow or arrows based on 2D state (x, y, yaw)

    All optional settings of matplotlib.pyplot.arrow can be used.
    - matplotlib.pyplot.arrow:
    https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.arrow.html

    Parameters
    ----------
    x : a float or array_like
        a value or a list of arrow origin x position.
    y : a float or array_like
        a value or a list of arrow origin y position.
    yaw : a float or array_like
        a value or a list of arrow yaw angle (orientation).
    arrow_length : a float (optional)
        arrow length. default is 1.0
    origin_point_plot_style : str (optional)
        origin point plot style. If None, not plotting.
    head_width : a float (optional)
        arrow head width. default is 0.1
    fc : string (optional)
        face color
    ec : string (optional)
        edge color
    """
    if not isinstance(x, float):
        for (i_x, i_y, i_yaw) in zip(x, y, yaw):
            plot_arrow(i_x, i_y, i_yaw, head_width=head_width,
                       fc=fc, ec=ec, **kwargs)
    else:
        plt.arrow(x, y,
                  arrow_length * np.cos(yaw),
                  arrow_length * np.sin(yaw),
                  head_width=head_width,
                  fc=fc, ec=ec,
                  **kwargs)
        if origin_point_plot_style is not None:
            plt.plot(x, y, origin_point_plot_style)


# Robotics functions
def lidar_to_coords(robot_pose, lidar_data, lidar_fov, lidar_res):
    '''
    Convert lidar data to coordinates
    '''
    
    lidar_x_list = []
    lidar_y_list = []
    
    values = np.linspace(lidar_fov/2, -lidar_fov/2, lidar_res)
    
    for i in range(512):
        lidar_x = robot_pose[0] + lidar_data[i]*(np.cos(robot_pose[2] + values[i]))
        lidar_y = robot_pose[1] + lidar_data[i]*(np.sin(robot_pose[2] + values[i]))
        lidar_x_list.append(lidar_x)
        lidar_y_list.append(lidar_y)
        
    return lidar_x_list, lidar_y_list

def heading_error(pos_1, pos_2, rot_matrix):
    '''
    find heading error for pure pursuit controller
    '''
    robot_heading = np.dot(rot_matrix, np.array([1, 0]))
    line = (np.array(pos_2) - np.array(pos_1))
    angle = np.arccos(np.dot(robot_heading, line)
                      / np.linalg.norm(line)
                      / np.linalg.norm(robot_heading))
    cross = np.cross(line, robot_heading)
    if cross > 0.0:
        angle = -angle

    return angle

def track_error(pos_1, pos_2, robot_pos):
    '''
    find track error for pure pursuit controller
    '''
    A = np.array(pos_1)
    B = np.array(pos_2)
    P = np.array([robot_pos[0], robot_pos[1], robot_pos[2]])
    n = np.cross([0, 0, 1], (B - A))
    distance = np.dot((P - A), n) / np.linalg.norm(n)
    return distance

def pure_pursuit(track_error, angle_error, track_param, angle_param):
    '''
    Simple pure pursuit controller
    '''
    
    ang_vel = (track_param * -track_error) + (angle_param * angle_error)

    return ang_vel

def get_ddr_wheel_velocities(linear_vel, angular_vel, wheel_radius, max_wheel_vel, wheel_base_length):
    
    """Set linear and angular velocities of the robot.

    Arguments:
    linear_velocity  -- Forward velocity in m/s.
    angular_velocity -- Rotational velocity rad/s. Positive direction is
                        counter-clockwise.
    """
    
    diff_vel = angular_vel * wheel_base_length / wheel_radius
    right_vel = (linear_vel + diff_vel) / wheel_radius
    left_vel = (linear_vel - diff_vel) / wheel_radius
    fastest_wheel = abs(max(right_vel, left_vel))
    if fastest_wheel > max_wheel_vel:
        left_vel = left_vel / fastest_wheel * max_wheel_vel
        right_vel = right_vel / fastest_wheel * max_wheel_vel
        
    return left_vel, right_vel