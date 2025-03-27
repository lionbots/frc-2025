package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.IMagicRotSubsystem;

/**
 * Command to magically toggle something's rotation with a PIDController, regardless of current rotation. 
 */
public class MagicRotCommand extends Command {
    private final IMagicRotSubsystem subsystem;
    // current target rotation
    private double targetRot;
    // subsystem minimum rotation. may work if out of 0 to 1 range, idk never tested
    private double minRot;
    // subsystem maximum rotation
    private double maxRot;
    private String name;
    private final boolean doPutData = true;
    
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

    @Override
    public void initialize() {
        // anything below or equal to half rotation will toggle to maxrotation
        // anything above half rotation will toggle to min rotation
        double halfRotation = this.minRot + (this.maxRot - this.minRot) / 2;
        this.targetRot = this.subsystem.getPivotPosition() < halfRotation ? maxRot : minRot;
        if (this.doPutData) {
            SmartDashboard.putNumber(name + " target rot", this.targetRot);
        }

        // set PID setpoint cuz PID gotta continue correcting errors
        // have to do in subsystem cuz command can't interrupt itself, eternal command much annoy
        this.subsystem.setSetpoint(this.targetRot);
    }

    @Override
    public boolean isFinished() {
        return this.subsystem.atSetPoint();
    }
}
