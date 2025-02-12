package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class PivotCommand extends Command{
private final IntakeSubsystem pivot;
private final Supplier<Double> speedFunction;

    public PivotCommand (IntakeSubsystem pivot, Supplier<Double> speedFunction) {
        this.pivot = pivot;
        this.speedFunction = speedFunction;
        //Use addRequirements() here to declare subsystem dependencies
        addRequirements(pivot);
    }
    public void execute() {
        pivot.setPivotSpeed(speedFunction.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
    return false;
}
}
    