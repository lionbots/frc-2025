// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.OuttakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

// An Outtake command that uses an Outtake subsystem. */
public class OuttakeCommand extends Command {
  private final OuttakeSubsystem outtake;
  private final Supplier<Double> actualSpeed;

  public OuttakeCommand (OuttakeSubsystem outtake, Supplier<Double> actualSpeed) {
    this.outtake = outtake;
    this.actualSpeed = actualSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(outtake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
