package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SendableDouble implements Sendable {
    private double thing;

    public SendableDouble(double thing) {
        this.thing = thing;
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