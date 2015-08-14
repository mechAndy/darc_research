# darc_research
This is all of the nodes that I created or worked with for my quadrotor research

## darc_crazyflie
This node was created to convert/map the command input, 'new_u', to the twist message, 'cmd_vel', that the crazyflie requires as in input.
- darc_cracyflie_node: Automated flight from a controller/trajectory generator
- darc_crazyflie_manual_node: Manual flight using an Xbox controller

## darc_custom_joy
This node was created to be used with darc_custom_quad and roscopter. It takes input from an Xbox controller and output, 'send_rc', for mavlink communications with the [3DR pixhawk (https://store.3drobotics.com/products/3dr-pixhawk?taxon_id=34)]

## darc_custom_quad
This is a simplified version of the darc_custom_joy. It no longer requires a custom version of roscopter to arm and disarm the quadrotor.  It takes input from an Xbox controller and output, 'send_rc', for mavlink communications with the [3DR pixhawk (https://store.3drobotics.com/products/3dr-pixhawk?taxon_id=34)]

## darc_position_hold
This is a standard PID position hold node. It was originally created by [damanfb(https://github.com/damanfb)] and I worked with Daman to update the file to its current form.

## darc_waypoint_generator
This node creates way points for trajectory following. In its current state it is meant to create simple paths to work with up to 3 quadrotors.  It was originally created by [damanfb(https://github.com/damanfb)] and I worked with Daman to update the file to its current form.


## darc_iterative_learn
This was created to use iterative learning control on the crazyflie nano quadrotors. The iterative controller is meant to be used with a standard controller. 
- ilc_waypoint_generator: This was originally created specifically for use with the iterative learning controller. This was eventually rolled into the  iterative_learn node.
- iterative_learn: This is the iterative learning controller and the way point generator.
