// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OuttakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

// An Outtake command that uses an Outtake subsystem. */
public class OuttakeCommand extends Command {
  private final OuttakeSubsystem launcher;
  //private final Supplier<Double> speedFunction;
  private final IntakeSubsystem intake;
  private final Supplier<Double> intakeSpeed;

  public OuttakeCommand (OuttakeSubsystem launcher, IntakeSubsystem intake, Supplier<Double> intakeSpeed) {
    this.launcher = launcher;
    this.intake = intake;
    this.intakeSpeed = intakeSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(outtake);
  }
    // Called every time the scheduler runs while the command is scheduled.

  public void execute(){
    launcher.setOuttakeSpeed(intakeSpeed.get());
    intake.setIntakeSpeed(intakeSpeed.get() * -1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    launcher.setOuttakeSpeed(0);
    intake.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
