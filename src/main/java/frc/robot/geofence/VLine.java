package frc.robot.geofence;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VLine implements GeofenceObject {
    double x;
    double radius;
    double buffer;
    String name;

    DoublePublisher distancePublisher;
    DoublePublisher dampedXMotionPublisher;
    DoublePublisher modifiedXMotionPublisher;

    public VLine(double x, double radius, double buffer) {
        this("", x, radius, buffer);
    }

    public VLine(String name, double x, double radius, double buffer) {
        this.name = name + "/";
        this.x = x;
        this.radius = radius;
        this.buffer = buffer;

        NetworkTableInstance defaultInstance = NetworkTableInstance.getDefault();
        this.distancePublisher = defaultInstance.getDoubleTopic("/geofence/" + this.name + "distance").publish();
        this.dampedXMotionPublisher = defaultInstance.getDoubleTopic("/geofence/" + this.name + "damped xMotion").publish();
        this.modifiedXMotionPublisher = defaultInstance.getDoubleTopic("/geofence/" + this.name + "final xMotion").publish();
    }

    @Override
    public Translation2d modifyMotion(Translation2d robotMotion, Translation2d robotPos, double robotRadius) {
        if (Math.abs(robotMotion.getX()) > 0.05) {
            double distance = Math.abs((this.x - this.radius) - (robotPos.getX() + robotRadius));
            double dampedXMotion = MathUtil.clamp(distance, 0, this.buffer) / this.buffer;
            double xMotion = Math.min(robotMotion.getX(), dampedXMotion);
            this.distancePublisher.set(distance);
            this.dampedXMotionPublisher.set(dampedXMotion);
            this.modifiedXMotionPublisher.set(xMotion);
            return new Translation2d(xMotion, robotMotion.getY());
        }
        return robotMotion;
    }
}
