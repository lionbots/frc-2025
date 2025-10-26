package frc.robot.geofence;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Geofencing point
 * 
 * A point with a buffer over which the robot will slow down and a radius at which the robot will stop. The robot's motion rotates around the point.
 */
public class Point implements GeofenceObject {
    double x;
    double y;
    /**
     * Limit that robot cannot pass
     */
    double radius;
    /**
     * Distance over which the robot's speed goes from normal to zero
     */
    double buffer;

    static final StructPublisher<Translation2d> projectionPublisher = NetworkTableInstance.getDefault().getStructTopic("/geofence/point/projection", Translation2d.struct).publish();
    static final StructPublisher<Translation2d> rejectionPublisher = NetworkTableInstance.getDefault().getStructTopic("/geofence/point/rejection", Translation2d.struct).publish();
    static final StructPublisher<Translation2d> newProjectionPublisher = NetworkTableInstance.getDefault().getStructTopic("/geofence/point/new projection", Translation2d.struct).publish();
    static final StructPublisher<Translation2d> modifiedMotionPublisher = NetworkTableInstance.getDefault().getStructTopic("/geofence/point/new projection", Translation2d.struct).publish();

    public Point(double x, double y, double radius, double buffer) {
        this.x = x;
        this.y = y;
        this.radius = radius;
        this.buffer = buffer;
    }

    @Override
    public Translation2d modifyMotion(Translation2d robotMotion, Translation2d robotPos, double robotRadius) {
        return Point.pointDamping(this.x, this.y, robotMotion, robotPos, robotRadius, this.radius, this.buffer);
    }

    /**
     * Rotates a velocity vector to glide around a point
     * @param x Field relative X coordinate of the point
     * @param y Field relative Y coordinate of the point
     * @param robotMotion Field relative robot velocity
     * @param robotPos Robot center position
     * @param robotRadius Robot radius
     * @param radius Radius around object that robot cannot pass
     * @param buffer Radius around object that robot begins to slow
     * @return Modified velocity
     */
    public static Translation2d pointDamping(double x, double y, Translation2d robotMotion, Translation2d robotPos, double robotRadius, double radius, double buffer) {
        // this method exists because line motion modification also needs point damping

        double robotSpeed = robotMotion.getNorm();
        double distanceToObject = Math.sqrt(Math.pow(robotPos.getX() - x, 2) + Math.pow(robotPos.getY() - y, 2));

        if (distanceToObject > robotRadius + buffer + radius || robotSpeed < 0.05) {
            return robotMotion;
        }
        
        double normalizedToObjectX = (x - robotPos.getX()) / distanceToObject;
        double normalizedToObjectY = (y - robotPos.getY()) / distanceToObject;
        double dotProduct = robotMotion.getX() * normalizedToObjectX + robotMotion.getY() * normalizedToObjectY;
        // component of robot motion vector toward the point
        double projectionX = normalizedToObjectX * dotProduct;
        double projectionY = normalizedToObjectY * dotProduct;
        // component of robot motion vector perpendicular to the line of sight to the point
        double rejectionX = robotMotion.getX() - projectionX;
        double rejectionY = robotMotion.getY() - projectionY;

        SmartDashboard.putNumber("robot speed", robotSpeed);
        SmartDashboard.putNumber("dot product", dotProduct);
        SmartDashboard.putNumber("distance to object", distanceToObject);
        projectionPublisher.set(new Translation2d(projectionX, projectionY));
        rejectionPublisher.set(new Translation2d(rejectionX, rejectionY));

        // reduce the magnitude of the projection to prevent the robot from going toward the point
        double projectionCoefficient = (distanceToObject - robotRadius - radius) / buffer;
        projectionX *= projectionCoefficient;
        projectionY *= projectionCoefficient;
        Translation2d modifiedMotion = new Translation2d(projectionX + rejectionX, projectionY + rejectionY);
        modifiedMotion = modifiedMotion.times(modifiedMotion.getNorm() / robotSpeed);

        SmartDashboard.putNumber("projection coefficient", projectionCoefficient);
        newProjectionPublisher.set(new Translation2d(projectionX, projectionY));
        modifiedMotionPublisher.set(modifiedMotion);

        return modifiedMotion;
    }
}
