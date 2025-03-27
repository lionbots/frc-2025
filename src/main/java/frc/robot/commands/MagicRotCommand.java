package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
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
    private PIDController pid;
    private String name;
    private final boolean doPutData;
    
    /**
     * Construct command
     * @param subsystem Subsystem to rotate
     * @param name Name to use for NetworkTables data keys
     * @param minRot "Minimum" rotation in amount of rotations from 0 to 1. Should be something that doesn't have to bother with wraparound
     * @param maxRot "Maximum" rotation in amount of rotations from 0 to 1. Should be something that doesn't have to bother with wraparound
     * @param kP Proportional coefficient as used by PIDController
     * @param kI Integral coefficient as used by PIDController
     * @param kD Derivative coefficient as used by PIDController
     */
    public MagicRotCommand(IMagicRotSubsystem subsystem, String name, double minRot, double maxRot, double kP, double kI, double kD, double tolerance) {
        this.subsystem = subsystem;
        this.minRot = minRot;
        this.maxRot = maxRot;
        this.pid = new PIDController(kP, kI, kD);
        // this.pid.setTolerance(tolerance);
        this.name = name;
        this.doPutData = name.length() > 0 && RobotBase.isSimulation();
        if (this.doPutData) {
            SmartDashboard.putData(name + " rot PID", pid);
        }
        addRequirements(subsystem);
    }

    /**
     * Construct command. PID coefficients default to 1, 0, 0.
     * @param subsystem Subsystem to rotate
     * @param name Name to use for NetworkTables data keys
     * @param startRot "Minimum" rotation in amount of rotations from 0 to 1. Should be something that doesn't have to bother with wraparound
     * @param endRot "Maximum" rotation in amount of rotations from 0 to 1. Should be something that doesn't have to bother with wraparound
     */
    public MagicRotCommand(IMagicRotSubsystem subsystem, String name, double startRot, double endRot) {
        // 0 kP means no movement
        // couldve used feedforward controller but me lazy
        this(subsystem, name, startRot, endRot, 1, 0, 0, 0.05);
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
    }

    @Override
    public void execute() {
        double currentRot = this.subsystem.getPivotPosition();
        this.subsystem.setPivotSpeed(this.pid.calculate(currentRot, this.targetRot));
        if (this.doPutData) {
            SmartDashboard.putNumber(name + " current rot", currentRot);
        }
    }

    @Override
    public boolean isFinished() {
        return this.pid.atSetpoint();
    }
}
