package frc.robot.commands;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivebaseSubsystem;

public class FollowTrajectoryCommand extends Command {
    private final LTVUnicycleController controller = new LTVUnicycleController(0.02);
    private final Timer timer = new Timer();
    private final DrivebaseSubsystem drivebase;
    private final Trajectory trajectory;
    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltsSecsPerMeter, DriveConstants.kaVoltSecsSquaredPerMeter);
    private final PIDController pid = new PIDController(DriveConstants.kPDriveVel, 0, 0);
    // stolen straight from RamseteCommand
    // probably used to move from previous point to this point
    private double prevTrajLeftSpeed = 0;
    private double prevTrajRightSpeed = 0;

    public FollowTrajectoryCommand(DrivebaseSubsystem drivebase, Trajectory trajectory) {
        this.drivebase = drivebase;
        this.trajectory = trajectory;
        SmartDashboard.putData("trajectory PID", this.pid);
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        this.timer.reset();
        this.timer.start();
    }

    @Override
    public void execute() {
        Trajectory.State reference = trajectory.sample(this.timer.get());
        ChassisSpeeds speeds = controller.calculate(this.drivebase.getPose(), reference);
        DifferentialDriveWheelSpeeds trajectoryWheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(speeds);
        DifferentialDriveWheelSpeeds currentWheelSpeeds = drivebase.getWheelSpeeds();
    
        // convert velocity to voltage
        // no clue how to actually do this, me just guessing
        // once again stolen from RamseteCommand
        double leftVoltage = feedForward.calculateWithVelocities(currentWheelSpeeds.leftMetersPerSecond, trajectoryWheelSpeeds.leftMetersPerSecond);
        double rightVoltage = feedForward.calculateWithVelocities(currentWheelSpeeds.rightMetersPerSecond, trajectoryWheelSpeeds.rightMetersPerSecond);
        if (this.prevTrajLeftSpeed != 0 || this.prevTrajRightSpeed != 0) {
            leftVoltage += pid.calculate(this.prevTrajLeftSpeed, trajectoryWheelSpeeds.leftMetersPerSecond);
            rightVoltage += pid.calculate(this.prevTrajRightSpeed, trajectoryWheelSpeeds.rightMetersPerSecond);
        }
        drivebase.voltageDrive(leftVoltage, rightVoltage);

        this.prevTrajLeftSpeed = trajectoryWheelSpeeds.leftMetersPerSecond;
        this.prevTrajRightSpeed = trajectoryWheelSpeeds.rightMetersPerSecond;
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.voltageDrive(0, 0);
        this.timer.stop();
    }

    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(this.trajectory.getTotalTimeSeconds());
    }
}
