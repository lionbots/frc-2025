package frc.robot.geofence;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class HLine implements GeofenceObject {
    double y;
    double radius;
    double buffer;
    String name;

    DoublePublisher distancePublisher;
    DoublePublisher dampedYMotion;
    DoublePublisher modifiedYMotionPublisher;

    public HLine(double y, double radius, double buffer) {
        this("", y, radius, buffer);
    }

    public HLine(String name, double y, double radius, double buffer) {
        this.name = name + "/";
        this.y = y;
        this.radius = radius;
        this.buffer = buffer;

        NetworkTableInstance defaultInstance = NetworkTableInstance.getDefault();
        this.distancePublisher = defaultInstance.getDoubleTopic("/geofence/" + this.name + "distance").publish();
        this.dampedYMotion = defaultInstance.getDoubleTopic("/geofence/" + this.name + "damped yMotion").publish();
        this.modifiedYMotionPublisher = defaultInstance.getDoubleTopic("/geofence/" + this.name + "final yMotion").publish();
    }

    @Override
    public Translation2d modifyMotion(Translation2d robotMotion, Translation2d robotPos, double robotRadius) {
        if (Math.abs(robotMotion.getY()) > 0.05) {
            double distance = (this.y - this.radius) - (robotPos.getY() + robotRadius);
            double dampedYMotion = MathUtil.clamp(distance, 0, this.buffer) / this.buffer;
            double yMotion = Math.min(robotMotion.getY(), dampedYMotion);
            this.distancePublisher.set(distance);
            this.dampedYMotion.set(dampedYMotion);
            this.modifiedYMotionPublisher.set(yMotion);
            return new Translation2d(robotMotion.getX(), yMotion);
        }
        return robotMotion;
    }
}
