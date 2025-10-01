package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SendableDouble implements Sendable {
    private double thing;

    public SendableDouble(double thing) {
        this.thing = thing;
    }

    public SendableDouble(double thing, String smartDashboardName) {
        this.thing = thing;
        SmartDashboard.putData(smartDashboardName, this);
    }

    public double getThing() {
        return this.thing;
    }

    public void setThing(double newThing) {
        this.thing = newThing;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("", this::getThing, this::setThing);
    }
}