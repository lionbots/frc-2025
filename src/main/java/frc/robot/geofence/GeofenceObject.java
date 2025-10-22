package frc.robot.geofence;

import edu.wpi.first.math.geometry.Translation2d;

public interface GeofenceObject {
    /**
     * Modify a robot's motion to avoid collision
     * @param robotMotion Robot field relative velocity
     * @param robotPos Robot center field relative coordinates
     * @param robotRadius Radius of a circle circumscribing the robot
     * @return XY velocities that cause the robot to avoid the obstacle
     */
    public Translation2d modifyMotion(Translation2d robotMotion, Translation2d robotPos, double robotRadius);
}
