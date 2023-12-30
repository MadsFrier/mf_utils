import numpy as np
import matplotlib.pyplot as plt

# Utility functions

# Plotting functions
def _plot_lidar_coords(lidar_coords_x, lidar_coords_y, marker='o', color='r'):
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
def _lidar_to_coords(robot_pose, lidar_data, lidar_fov, lidar_res):
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