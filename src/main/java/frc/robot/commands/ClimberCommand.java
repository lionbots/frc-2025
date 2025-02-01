// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class ClimberCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem climber;
  private final Supplier<Double> speed;

  /**
   * Creates a new ClimberCommand.
   *
   * @param climber - The subsystem used by this command.
   * @param speed - The function for the climber's speed. 
   */
  public ClimberCommand(ClimberSubsystem climber, Supplier<Double> speed) {
    this.climber = climber;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  // Sets the speed of the climber. 
  @Override
  public void execute() {
    climber.setSpeed(speed.get());
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
