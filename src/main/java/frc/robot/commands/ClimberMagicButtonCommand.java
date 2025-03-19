package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberMagicButtonCommand extends Command {
    private final ClimberSubsystem climber;
    private double startRot;
    private final double magicThreshold = 0.0069;
    private final double halfRotation = -0.125;

    public ClimberMagicButtonCommand(ClimberSubsystem climber) {
        this.climber = climber;
        this.startRot = this.climber.getPosition();
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        this.startRot = this.climber.getPosition();
    }

    @Override
    public void execute() {
        // this damn thing could start in the middle so move toward closest or whatever
        this.climber.setSpeed(startRot < halfRotation ? 1 : -1);
    }

    @Override
    public boolean isFinished() {
        // return false;
        double encoderPos = this.climber.getPosition();
        return (startRot >= halfRotation && Math.abs(-0.25 - encoderPos) < magicThreshold) || (startRot < halfRotation && Math.abs(encoderPos) < magicThreshold);
    }
}
