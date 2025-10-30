package frc.robot.geofence;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Axis aligned line
 * 
 * An infintely long line aligned with an axis. Has buffer over which the robot will slow down and a radius at which the robot will stop.
 */
public class AALine implements GeofenceObject {
    double axis;
    double radius;
    double buffer;
    boolean horizontal;
    boolean robotPosGreater;

    DoublePublisher distancePublisher;
    DoublePublisher dampedMotionPublisher;
    DoublePublisher modifiedMotionPublisher;

    /** 
     * Construct an infinitely long axis aligned line
     * @param axis Y coordinate of the line if horizontal or X coordinate if not
     * @param radius Radius at which the robot will stop
     * @param buffer Buffer over which the robot slows down
     * @param horizontal Whether the line is horizontal
     * @param robotPosGreater Whether the robot's position on the axis is greater than the line's
     */
    public AALine(double axis, double radius, double buffer, boolean horizontal, boolean robotPosGreater) {
        this("", axis, radius, buffer, horizontal, robotPosGreater);
    }

    /** 
     * Construct an infinitely long axis aligned line
     * @param name Name of the line, used for NetworkTables topics
     * @param axis Y coordinate of the line if horizontal or X coordinate if not
     * @param radius Radius at which the robot will stop
     * @param buffer Buffer over which the robot slows down
     * @param horizontal Whether the line is horizontal
     * @param robotPosGreater Whether the robot's position on the axis is greater than the line's
     */
    public AALine(String name, double axis, double radius, double buffer, boolean horizontal, boolean robotPosGreater) {
        this.axis = axis;
        this.radius = radius;
        this.buffer = buffer;
        this.horizontal = horizontal;
        this.robotPosGreater = robotPosGreater;
        
        name += "/";
        NetworkTableInstance defaultInstance = NetworkTableInstance.getDefault();
        this.distancePublisher = defaultInstance.getDoubleTopic("/geofence/" + name + "distance").publish();
        this.dampedMotionPublisher = defaultInstance.getDoubleTopic("/geofence/" + name + "damped " + (this.horizontal ? "yMotion" : "xMotion")).publish();
        this.modifiedMotionPublisher = defaultInstance.getDoubleTopic("/geofence/" + name + "final " + (this.horizontal ? "yMotion" : "xMotion")).publish();
    }

    @Override
    public Translation2d modifyMotion(Translation2d robotMotion, Translation2d robotPos, double robotRadius) {
        double robotAxisPos = this.horizontal ? robotPos.getY() : robotPos.getX();
        double robotAxisMotion = this.horizontal ? robotMotion.getY() : robotMotion.getX();
        if (Math.signum(robotAxisMotion) != Math.signum(this.axis - robotAxisPos)) {
            return robotMotion;
        }
        double distance, dampedMotion, motion;
        if (this.robotPosGreater) {
            distance = (this.axis + this.radius) - (robotAxisPos - robotRadius);
            // the closer the robot's radius approaches the line's radius, the slow the robot moves toward the line
            // everything here is negative so spam unary negation and max() and clamp() until something appears to work
            dampedMotion = MathUtil.clamp(distance, -this.buffer, 0) / this.buffer;
            motion = Math.max(robotAxisMotion, dampedMotion);
        } else {
            distance = (this.axis - this.radius) - (robotAxisPos + robotRadius);
            dampedMotion = MathUtil.clamp(distance, 0, this.buffer) / this.buffer;
            motion = Math.min(robotAxisMotion, dampedMotion);
        }
        this.distancePublisher.set(distance);
        this.dampedMotionPublisher.set(dampedMotion);
        this.modifiedMotionPublisher.set(motion);
        return this.horizontal ? new Translation2d(robotMotion.getX(), motion) : new Translation2d(motion, robotMotion.getY());
    }
}
