// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import edu.wpi.first.wpilibj2.command.*;
import java.util.function.Supplier;

// An Intake command that uses an Intake subsystem.
public class IntakeCommand extends Command {
    private final IntakeSubsystem intake;
    private final OuttakeSubsystem launcher;
    private final Supplier<Double> intakeSpeed;

  public IntakeCommand(IntakeSubsystem intake, OuttakeSubsystem launcher, Supplier<Double> intakeSpeed) {
    this.intake = intake;
    this.intakeSpeed = intakeSpeed;
    this.launcher = launcher;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakeSpeed(intakeSpeed.get());
    launcher.setOuterSpeed(intakeSpeed.get());
    launcher.setInnerSpeed(intakeSpeed.get());
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
