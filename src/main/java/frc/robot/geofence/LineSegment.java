package frc.robot.geofence;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.geofence.Point.PointDampingPublishers;

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

    // DoublePublisher dotProductPublisher = null;
    StructPublisher<Translation2d> closestPointPublisher = null;
    PointDampingPublishers pointDampingPublishers = null;

    /**
     * Constructs a geofence line segment
     * @param aX X coordinate of one segment endpoint
     * @param aY Y coordinate of one segment endpoint
     * @param bX X coordinate of other segment endpoint
     * @param bY Y coordinate of other segment endpoint
     * @param radius Radius around line where robot stops
     * @param buffer Buffer around radius where robot starts to slow down
     */
    public LineSegment(double aX, double aY, double bX, double bY, double radius, double buffer) {
        this.aX = aX;
        this.aY = aY;
        this.bX = bX;
        this.bY = bY;
        this.dXab = this.bX - aX;
        this.dYab = this.bY - aY;
        this.lenSquared = this.dXab * this.dXab + this.dYab * this.dYab;
        this.radius = radius;
        this.buffer = buffer;
    }

    /**
     * Constructs a geofence line segment
     * @param name NetworkTables topic prefix
     * @param aX X coordinate of one segment endpoint
     * @param aY Y coordinate of one segment endpoint
     * @param bX X coordinate of other segment endpoint
     * @param bY Y coordinate of other segment endpoint
     * @param radius Radius around line where robot stops
     * @param buffer Buffer around radius where robot starts to slow down
     */
    public LineSegment(String name, double aX, double aY, double bX, double bY, double radius, double buffer) {
        this(aX, aY, bX, bY, radius, buffer);
        this.initializeNT(name);
    }
    
    public void initializeNT(String name) {
        name += "/";
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        // this.dotProductPublisher = instance.getDoubleTopic("/geofence/" + name + "normalizedDotProduct").publish();
        this.closestPointPublisher = instance.getStructTopic("/geofence/" + name + "closestPoint", Translation2d.struct).publish();
        instance.getStructTopic("/geofence/" + name + "point one", Translation2d.struct).publish().set(new Translation2d(this.aX, this.aY));
        instance.getStructTopic("/geofence/" + name + "point two", Translation2d.struct).publish().set(new Translation2d(this.bX, this.bY));
        this.pointDampingPublishers = new PointDampingPublishers("/geofence/" + name);
    }

    public Translation2d closestPoint(double x, double y) {
        double dot = (((x - this.aX) * this.dXab) + ((y - this.aY) * this.dYab)) / this.lenSquared;
        // closest point on the line, clamped to be within the segment
        double closestX = MathUtil.clamp(this.aX + this.dXab * dot, Math.min(this.aX, this.bX), Math.max(this.aX, this.bX));
        double closestY = MathUtil.clamp(this.aY + this.dYab * dot, Math.min(this.aY, this.bY), Math.max(this.aY, this.bY));
        return new Translation2d(closestX, closestY);
    }
    
    @Override
    public Translation2d modifyMotion(Translation2d robotMotion, Translation2d robotPos, double robotRadius) {
        Translation2d closestPoint = this.closestPoint(robotPos.getX(), robotPos.getY());
        if (this.closestPointPublisher != null) {
            this.closestPointPublisher.set(closestPoint);
        }
        return Point.pointDamping(closestPoint.getX(), closestPoint.getY(), robotMotion, robotPos, robotRadius, this.radius, this.buffer, this.pointDampingPublishers);
    }
}
