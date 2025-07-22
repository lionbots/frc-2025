package frc.robot.commands;

import frc.robot.subsystems.OuttakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class OuttakePivotCommand extends Command {
    private final OuttakeSubsystem pivot;
    private final Supplier<Double> speedFunction;

    public OuttakePivotCommand (OuttakeSubsystem pivot, Supplier<Double> speedFunction) {
        this.pivot = pivot;
        this.speedFunction = speedFunction;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(pivot);
    }

    // Called every time the scheduler runs while the command is scheduled.
    public void execute(){
        pivot.setPivotSpeed(speedFunction.get() * 0.5);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        pivot.setPivotSpeed(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
