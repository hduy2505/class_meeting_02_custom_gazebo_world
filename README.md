# Warmup project
 
## Robot Behaviors

### Driving in a Square
**High-Level Description:** This behavior enables the robot to drive in a square path. The approach involves sequential 90-degree turns and straight-line movements equal in length.

**Code Explanation:**
- `move_forward()`: Commands the robot to move forward a specified distance.
- `turn()`: Rotates the robot 90 degrees.
- `stop()`: Stop the robot to prevent the deviation.

### Wall Follower
**High-Level Description:** The wall follower behavior is designed to keep the robot at a constant distance from a wall. It uses sensors to maintain the distance and navigate along the wall.

**Code Explanation:**
The scan_callback method is called whenever a new message is received on the /scan topic. It calculates the minimum distance to an obstacle in front of and to the right of the robot. Then, based on these distances, it sets the linear and angular velocities of the robot to make it follow the wall.

### Person Follower
**High-Level Description:** This behavior allows the robot to follow a person. It uses Lidar sensor to identify and track the person's movement.

**Code Explanation:**
The code use the scan_call back method to retrieve the range of the robot to an object. If the range is to close, the robot stop moving. If the robot detect an object in range, it turn until the object is in front of it and then move forward

## Challenges
In programming these robot behaviors, the main challenges included sensor calibration, ensuring precise movements, and developing robust algorithms for person tracking. Overcoming these required iterative testing and refinement of the code.

## Future Work
With more time, improvements would include enhancing the precision of movements, optimizing the algorithms for faster response times, and integrating machine learning for better performance.

## Takeaways
- **Testing and Iteration:** Frequent testing and iteration are crucial for refining robot behaviors. Each test provided insights that guided the next round of improvements.
- **Sensor Reliability:** Ensuring sensor reliability is key to consistent robot performance. This project highlighted the importance of calibrating and maintaining sensors.

```