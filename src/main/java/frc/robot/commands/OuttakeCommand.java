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
  private final Supplier<Double> speedFunction;

  public OuttakeCommand (OuttakeSubsystem outtake, Supplier<Double> speedFunction) {
    this.outtake = outtake;
    this.speedFunction = speedFunction;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(outtake);
  }
    // Called every time the scheduler runs while the command is scheduled.

  public void execute(){
    outtake.setOuttakeSpeed(speedFunction.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
