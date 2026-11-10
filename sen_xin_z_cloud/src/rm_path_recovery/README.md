PathTranslate uses the active navigation path, projects the robot onto its nearest
segment, chooses a point 0.5 m ahead along the path, and translates up to 0.3 m
toward it at 0.1 m/s. It commands zero angular velocity. Each cycle checks the
published local costmap and footprint along the translation. A blocked path,
missing pose, absent/remote path, or timeout fails the action and commands zero.

The base must support both cmd_vel.linear.x and cmd_vel.linear.y. Parameters in
the XML can be adjusted after testing on the robot.
