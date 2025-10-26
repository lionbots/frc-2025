package frc.robot.geofence;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AALine implements GeofenceObject {
    double axis;
    double radius;
    double buffer;
    String name;
    boolean horizontal;

    DoublePublisher distancePublisher;
    DoublePublisher dampedMotionPublisher;
    DoublePublisher modifiedMotionPublisher;

    /** 
     * Construct an infinitely long axis aligned line
     * @param axis Y coordinate of the line if horizontal or X coordinate if not
     * @param radius Radius at which the robot will stop
     * @param buffer Buffer over which the robot slows down
     * @param horizontal Whether the line is horizontal
     */
    public AALine(double axis, double radius, double buffer, boolean horizontal) {
        this("", axis, radius, buffer, horizontal);
    }

    public AALine(String name, double axis, double radius, double buffer, boolean horizontal) {
        this.name = name + "/";
        this.axis = axis;
        this.radius = radius;
        this.buffer = buffer;
        this.horizontal = horizontal;

        NetworkTableInstance defaultInstance = NetworkTableInstance.getDefault();
        this.distancePublisher = defaultInstance.getDoubleTopic("/geofence/" + this.name + "distance").publish();
        this.dampedMotionPublisher = defaultInstance.getDoubleTopic("/geofence/" + this.name + "damped " + (this.horizontal ? "yMotion" : "xMotion")).publish();
        this.modifiedMotionPublisher = defaultInstance.getDoubleTopic("/geofence/" + this.name + "final " + (this.horizontal ? "yMotion" : "xMotion")).publish();
    }

    @Override
    public Translation2d modifyMotion(Translation2d robotMotion, Translation2d robotPos, double robotRadius) {
        if ((this.horizontal && Math.signum(robotMotion.getY()) != Math.signum(this.axis - robotPos.getY())) || (!this.horizontal && Math.signum(robotMotion.getX()) != Math.signum(this.axis - robotPos.getX()))) {
            return robotMotion;
        }
        double distance = (this.axis - this.radius) - ((this.horizontal ? robotPos.getY() : robotPos.getX()) + robotRadius);
        double dampedMotion = MathUtil.clamp(distance, 0, this.buffer) / this.buffer;
        double motion = Math.min(this.horizontal ? robotPos.getY() : robotMotion.getX(), dampedMotion);
        this.distancePublisher.set(distance);
        this.dampedMotionPublisher.set(dampedMotion);
        this.modifiedMotionPublisher.set(motion);
        return this.horizontal ? new Translation2d(robotMotion.getX(), motion) : new Translation2d(motion, robotMotion.getY());
    }
}
