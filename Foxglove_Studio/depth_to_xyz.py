
import math

# depth - pixel depth value, P_X - pixel x value, P_Y - pixel y value
# z - vertical, x - hrizontal depth, y - hrizontal width 
def yxz(depth, p_X, p_Y):
    x_theta = math.atan( 320/(abs(p_X)*1.732))
    z_theta = math.atan( 320/(abs(p_Y)*1.732))
    y = depth * math.tan( x_theta)
    x = depth
    z = - depth * math.tan( z_theta)

    if ( p_X <= 0):
        x = -x
    if ( p_Y <= 0):
        z = -z

    # print(y, x, z)
    return y,x,z


print(yxz(585,300,300))