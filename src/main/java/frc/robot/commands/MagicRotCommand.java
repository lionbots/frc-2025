package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.IMagicRotSubsystem;

/**
 * Command to magically toggle something's rotation with a PIDController, regardless of current rotation. 
 */
public class MagicRotCommand extends Command {
    private final IMagicRotSubsystem subsystem;
    // subsystem minimum rotation. may work if out of 0 to 1 range, idk never tested
    private double minRot;
    // subsystem maximum rotation
    private double maxRot;
    private Double continuousMax = null;
    private String name;
    
    /**
     * Construct command
     * @param subsystem Subsystem to rotate
     * @param name Name to use for NetworkTables data keys
     * @param minRot "Minimum" rotation in amount of rotations from 0 to 1. Should be something that doesn't have to bother with wraparound
     * @param maxRot "Maximum" rotation in amount of rotations from 0 to 1. Should be something that doesn't have to bother with wraparound
     */
    public MagicRotCommand(IMagicRotSubsystem subsystem, String name, double minRot, double maxRot) {
        this.subsystem = subsystem;
        // math.min() cuz i dont trust whoever uses this command (myself)
        this.minRot = Math.min(minRot, maxRot);
        this.maxRot = Math.max(minRot, maxRot);
        // this.pid.setTolerance(tolerance);
        this.name = name;
        addRequirements(subsystem);
    }

    public MagicRotCommand enableContinuous(Double max) {
        this.continuousMax = max;
        return this;
    }

    @Override
    public void initialize() {
        double targetRot;
        double pivotPos = this.subsystem.getPivotPosition();
        if (this.continuousMax == null) {
            // anything below or equal to half rotation will toggle to maxrotation
            // anything above half rotation will toggle to min rotation
            double halfRotation = this.minRot + (this.maxRot - this.minRot) / 2;
            targetRot = pivotPos < halfRotation ? maxRot : minRot;
        } else {
            double minRotToPivotDist = this.minRot + (pivotPos > this.minRot ? this.continuousMax : 0) - pivotPos;
            double maxRotToPivotDist = this.maxRot + (pivotPos > this.maxRot ? this.continuousMax : 0) - pivotPos;
            targetRot = minRotToPivotDist > maxRotToPivotDist ? this.minRot : this.maxRot;
        }
        SmartDashboard.putNumber(name + " target rot", targetRot);

        // set PID setpoint cuz PID gotta continue correcting errors
        // have to do in subsystem cuz command can't interrupt itself, eternal command much annoy
        this.subsystem.setSetpoint(targetRot);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
