package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberMagicButtonCommand extends Command {
    private final ClimberSubsystem climber;
    private double targetRot;
    private final double startPos = 0;
    private final double endPos = 0.25;
    private final double halfRotation = startPos + (endPos - startPos) / 2;

    public ClimberMagicButtonCommand(ClimberSubsystem climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        this.targetRot = this.climber.getPosition() < halfRotation ? endPos : startPos;
    }

    @Override
    public void execute() {
        this.climber.setSpeed(this.climber.toRot(this.targetRot));
    }

    @Override
    public boolean isFinished() {
        return this.climber.atSetPoint();
    }
}
