package frc;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Subsystem used by MagicRotCommand
 */
public interface IMagicRotSubsystem extends Subsystem {
    // this interface should really be a mixin cuz all them subsystems are basically the same
    // me no know how tho
    public double getPivotPosition();
    public boolean atSetPoint();
    /**
     * Set the setpoint the subsystem will attempt to go to. If NaN then clear setpos
     */
    public void setSetpoint(double pos);
}
