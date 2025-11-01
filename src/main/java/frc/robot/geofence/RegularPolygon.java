package frc.robot.geofence;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RegularPolygon implements GeofenceObject {
    private final LineSegment[] edges;
    private final double[][] points;
    double centerX;
    double centerY;
    double polygonRadius;
    double radius;
    double buffer;

    BooleanPublisher inPolgyon = null;
    IntegerPublisher closestEdgeIdx = null;

    /**
     * Construct a polygon geofencing object
     * @param centerX X coordinate of the center of the polgyon
     * @param centerY Y coordinate of the center of the polygon
     * @param polygonRadius Radius of a circle circumscribing the polygon
     * @param numPoints Number of points in the polygon
     * @param rotation Rotation of the polygon
     * @param radius Radius around polygon at which robot stops
     * @param buffer Buffer around radius at which robot slows
     */
    public RegularPolygon(double centerX, double centerY, double polygonRadius, int numPoints, double rotation, double radius, double buffer) {
        this.centerX = centerX;
        this.centerY = centerY;
        this.polygonRadius = polygonRadius;
        this.radius = radius;
        this.buffer = buffer;

        this.edges = new LineSegment[numPoints];
        this.points = new double[numPoints][2];
        double previousPointX = polygonRadius * Math.cos(rotation) + centerX;
        double previousPointY = polygonRadius * Math.sin(rotation) + centerY;
        for (int i = 1; i < numPoints; i++) {
            this.points[i - 1][0] = previousPointX;
            this.points[i - 1][1] = previousPointY;
            double currentPointX = polygonRadius * Math.cos(rotation + 2 * Math.PI / numPoints * i) + centerX;
            double currentPointY = polygonRadius * Math.sin(rotation + 2 * Math.PI / numPoints * i) + centerY;
            this.edges[i - 1] = new LineSegment(previousPointX, previousPointY, currentPointX, currentPointY, radius, buffer);
            previousPointX = currentPointX;
            previousPointY = currentPointY;
        }
        this.edges[numPoints - 1] = new LineSegment(previousPointX, previousPointY, polygonRadius * Math.cos(rotation) + centerX, polygonRadius * Math.sin(rotation) + centerY, radius, buffer);
        this.points[numPoints - 1][0] = previousPointX;
        this.points[numPoints - 1][1] = previousPointY;
    }

    public RegularPolygon(String name, double centerX, double centerY, double polygonRadius, int numPoints, double rotation, double radius, double buffer) {
        this(centerX, centerY, polygonRadius, numPoints, rotation, radius, buffer);

        for (int i = 0; i < numPoints; i++) {
            this.edges[i].initializeNT(name + "/line" + i);
        }
    
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        this.inPolgyon = instance.getBooleanTopic("/geofence/" + name + "/inPolygon").publish();
        this.closestEdgeIdx = instance.getIntegerTopic("/geofence/" + name + "/closestEdgeIdx").publish();
        
    }

    @Override
    public Translation2d modifyMotion(Translation2d robotMotion, Translation2d robotPos, double robotRadius) {
        // if the robot is inside the polygon, push it away from the polygon's center
        if (this.pointPolygonCollide(robotPos.getX(), robotPos.getY(), this.points)) {
            if (this.inPolgyon != null) {
                this.inPolgyon.set(true);
                this.closestEdgeIdx.set(-1);
            }
            return Point.pointDamping(this.centerX, this.centerY, robotMotion, robotPos, robotRadius, this.polygonRadius, this.buffer);
        }

        // else push the robot away from the closest line
        double closestLineDist = robotPos.getDistance(this.edges[0].closestPoint(robotPos.getX(), robotPos.getY()));
        int closestLineIdx = 0;
        for (int i = 1; i < this.edges.length; i++) {
            double distance = robotPos.getDistance(this.edges[i].closestPoint(robotPos.getX(), robotPos.getY()));
            if (distance < closestLineDist) {
                closestLineIdx = i;
                closestLineDist = distance;
            }
        }

        if (this.inPolgyon != null) {
            this.inPolgyon.set(false);
            this.closestEdgeIdx.set(closestLineIdx);
        }

        return this.edges[closestLineIdx].modifyMotion(robotMotion, robotPos, robotRadius);
    }
    
    /**
     * Whether three points are in counterclockwise orientation
     * @param p1x X coordinate of first point
     * @param p1y Y coordinate of first point
     * @param p2x X coordinate of second point
     * @param p2y Y coordinate of second point
     * @param p3x X coordinate of third point
     * @param p3y Y coordinate of third point
     * @return Counterblockwise
     */
    private boolean pointsCounterclockwise(double p1x, double p1y, double p2x, double p2y, double p3x, double p3y) {
        // black magic from one of my games, i have no clue how this works
        return (p3y - p1y) * (p2x - p1x) > (p2y - p1y) * (p3x - p1x);
    }

    /**
     * Line segment intersection
     * @param ax1 X coordinate of first point of first segment
     * @param ay1 Y coordinate of first point of first segment
     * @param bx1 X coordinate of second point of first segment
     * @param by1 Y coordinate of second point of first segment
     * @param ax2 X coordinate of first point of second segment
     * @param ay2 Y coordinate of first point of second segment
     * @param bx2 X coordinate of second point of second segment
     * @param by2 Y coordinate of second point of second segment
     * @return Whether the segments intersect
     */
    private boolean segmentSegmentIntersect(double ax1, double ay1, double bx1, double by1, double ax2, double ay2, double bx2, double by2) {
        // intersecting segments have their points oriented in a certain way but i forgor how
        return pointsCounterclockwise(ax1, ay1, ax2, ay2, bx2, by2) != pointsCounterclockwise(bx1, by1, ax2, ay2, bx2, by2) && pointsCounterclockwise(ax1, ay1, bx1, by1, ax2, ay2) != pointsCounterclockwise(ax1, ay1, bx1, by1, bx2, by2);
    }

    /**
     * Point polygon collision
     * @param x X coordinate of point
     * @param y Y coordinate of point
     * @param polygonPoints (x, y) of points in polygon
     * @return Whether the point is inside the polygon, excluding the polygon's edges
     */
    private boolean pointPolygonCollide(double x, double y, double[][] polygonPoints) {
        // create lines from (x, y) to certain faraway points
        // if the point is inside the polygon and the polygon is "normal", there will be an odd number of intersections
        // may not be the most efficient because the game this is copied from had certain needs (ridiculous unit tests) that necessitated weird code\
        SmartDashboard.putNumber("robotX", x);
        SmartDashboard.putNumber("robotY", y);
        final double[][] multipliers = {{694, 694}, {694, -694}, {694, 420}};
        for (int i = 0; i < multipliers.length; i++) {
            double endX = (x == 0 ? 1 : x) * multipliers[i][0];
            double endY = (y == 0 ? 1 : y) * multipliers[i][1];
            int numIntersections = 0;
            SmartDashboard.putNumber("endX" + i, endX);
            SmartDashboard.putNumber("endY" + i, endY);
            for (int j = 0; j < polygonPoints.length; j++) {
                // String key = i + " " + j + " (" + ((double) Math.round(x * 100) / 100) + ", " + ((double) Math.round(y * 100) / 100) + ")->(" + ((double) Math.round(endX * 100) / 100) + ", " + (Math.round(endY * 100) / 100) + ")x(" + ((double) Math.round(polygonPoints[j][0] * 100) / 100) + ", " + ((double) Math.round(polygonPoints[j][1] * 100) / 100) + ")->(" + ((double) Math.round(polygonPoints[(j + 1) % polygonPoints.length][0] * 100) / 100) + ", " + ((double) Math.round(polygonPoints[(j + 1) % polygonPoints.length][1] * 100) / 100) + ")";
                if (segmentSegmentIntersect(x, y, endX, endY, polygonPoints[j][0], polygonPoints[j][1], polygonPoints[(j + 1) % polygonPoints.length][0], polygonPoints[(j + 1) % polygonPoints.length][1])) {
                    numIntersections++;
                    // SmartDashboard.putBoolean(key, true);
                } else {
                    // SmartDashboard.putBoolean(key, false);
                }
            }
            SmartDashboard.putNumber("intersectionCount" + i, numIntersections);
            if (numIntersections % 2 != 0) {
                return true;
            }
        }
        return false;
    }
}
