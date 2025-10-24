package frc.robot.geofence;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GeofencePoint implements GeofenceObject {
    double x;
    double y;
    /**
     * Distance over which the robot's speed goes from normal to zero
     */
    double buffer;

    public GeofencePoint(double x, double y, double buffer) {
        this.x = x;
        this.y = y;
        this.buffer = buffer;
    }

    @Override
    public Translation2d modifyMotion(Translation2d robotMotion, Translation2d robotPos, double robotRadius) {
        return GeofencePoint.pointDamping(this.x, this.y, robotMotion, robotPos, robotRadius, this.buffer);
    }

    /**
     * Rotates a velocity vector to glide around a point
     * @param x Field relative X coordinate of the point
     * @param y Field relative Y coordinate of the point
     * @param robotMotion Field relative robot velocity
     * @param robotPos Robot center position
     * @param robotRadius Robot radius
     * @return Modified velocity
     */
    public static Translation2d pointDamping(double x, double y, Translation2d robotMotion, Translation2d robotPos, double robotRadius, double buffer) {
        // this method exists because line motion modification also needs point damping

        double robotSpeed = robotMotion.getNorm();
        double distanceToObject = Math.sqrt(Math.pow(robotPos.getX() - x, 2) + Math.pow(robotPos.getY() - y, 2));

        if (distanceToObject > robotRadius + buffer || robotSpeed < 0.05) {
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

        SmartDashboard.putString("robot motion", "(" + robotMotion.getX() + ", " + robotMotion.getY() + ")");
        SmartDashboard.putString("projection", "(" + projectionX + ", " + projectionY + ")");
        SmartDashboard.putString("rejection", "(" + rejectionX + ", "+ rejectionY + ")");
        SmartDashboard.putNumber("robot speed", robotSpeed);
        SmartDashboard.putNumber("dot product", dotProduct);

        // reduce the magnitude of the projection to prevent the robot from going toward the point
        double projectionCoefficient = robotSpeed - dotProduct;
        projectionX *= projectionCoefficient;
        projectionY *= projectionCoefficient;

        SmartDashboard.putString("new projection", "(" + projectionX + ", " + projectionY + ")");

        Translation2d modifiedMotion = new Translation2d(projectionX + rejectionX, projectionY + rejectionY);
        return modifiedMotion.times(modifiedMotion.getNorm() / robotSpeed);
    }
}
