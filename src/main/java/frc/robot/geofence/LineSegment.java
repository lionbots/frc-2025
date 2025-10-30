package frc.robot.geofence;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public class LineSegment implements GeofenceObject {
    final double aX;
    final double aY;
    final double bX;
    final double bY;
    final double dXab;
    final double dYab;
    final double lenSquared;
    final double radius;
    final double buffer;

    DoublePublisher dotProductPublisher;
    StructPublisher<Translation2d> closestPoint;

    /**
     * Constructs a geofence line segment
     * @param aX X coordinate of one segment endpoint
     * @param aY Y coordinate of one segment endpoint
     * @param bX X coordinate of other segment endpoint
     * @param bY Y coordinate of other segment endpoint
     * @param
     */
    public LineSegment(String name, double aX, double aY, double bX, double bY, double radius, double buffer) {
        this.aX = aX;
        this.aY = aY;
        this.bX = bX;
        this.bY = bY;
        this.dXab = this.bX - aX;
        this.dYab = this.bY - aY;
        this.lenSquared = this.dXab * this.dXab + this.dYab * this.dYab;
        this.radius = radius;
        this.buffer = buffer;

        name += "/";
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        this.dotProductPublisher = instance.getDoubleTopic("/geofence/" + name + "normalizedDotProduct").publish();
        this.closestPoint = instance.getStructTopic("/geofence/" + name + "closestPoint", Translation2d.struct).publish();
        instance.getStructTopic("/geofence/" + name + "point one", Translation2d.struct).publish().set(new Translation2d(this.aX, this.aY));
        instance.getStructTopic("/geofence/" + name + "point two", Translation2d.struct).publish().set(new Translation2d(this.bX, this.bY));
    }

    @Override
    public Translation2d modifyMotion(Translation2d robotMotion, Translation2d robotPos, double robotRadius) {
        // how far along the line the closest point to the robot is
        // calculated by finding the intersection of the segment and the line perpendicular to the segment that goes through the robot
        // intersection of segment and robot motion may be better
        double dot = (((robotPos.getX() - this.aX) * this.dXab) + ((robotPos.getY() - this.aY) * this.dYab)) / this.lenSquared;
        // closest point on the line, clamped to be within the segment
        double closestX = MathUtil.clamp(this.aX + this.dXab * dot, Math.min(this.aX, this.bX), Math.max(this.aX, this.bX));
        double closestY = MathUtil.clamp(this.aY + this.dYab * dot, Math.min(this.aY, this.bY), Math.max(this.aY, this.bY));
        this.dotProductPublisher.set(dot);
        this.closestPoint.set(new Translation2d(closestX, closestY));
        return Point.pointDamping(closestX, closestY, robotMotion, robotPos, robotRadius, this.radius, this.buffer);
    }
}
