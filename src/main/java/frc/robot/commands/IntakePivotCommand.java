package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class IntakePivotCommand extends Command {
    private final IntakeSubsystem pivot;
    private final Supplier<Double> speedFunction;
        
    public IntakePivotCommand(IntakeSubsystem pivot, Supplier<Double> speedFunction) {
        this.pivot = pivot;
        this.speedFunction = speedFunction;
        //Use addRequirements() here to declare subsystem dependencies
        addRequirements(pivot);
    }

    public void execute() {
        double speed = this.speedFunction.get();
        if (speed != 0) {
            // MagicRotCommand makes the subsystem PID go toward its setpoint forever
            // this command should override that, so clear the setpoint
            // didnt do this in initialize() cuz i have no clue when thats called but in my experience it dont allow this command to interrupt magic rotation
            this.pivot.setSetpoint(null);
            pivot.setPivotSpeed(speed * 0.1);
        }
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
