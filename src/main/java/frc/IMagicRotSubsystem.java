package frc;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Subsystem used by MagicRotCommand
 */
public interface IMagicRotSubsystem extends Subsystem {
    public void setPivotSpeed(double speed);
    /**
     * Get pivot's position
     * @return Amount of revolutions ideally in range 0 to 1
     */
    public double getPivotPosition();
}
