// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DrivebaseSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class FieldCentricDriveCommand extends Command {

  private final Supplier<Double> speedSupplier, xFunction, yFunction;
  private final DrivebaseSubsystem drivebase;
  private final PIDController pid = new PIDController(0, 0, 0);

  public FieldCentricDriveCommand(DrivebaseSubsystem subsystem, Supplier<Double> xFunction, Supplier<Double> yFunction, Supplier<Double> speedSupplier) {
    drivebase = subsystem;
    this.speedSupplier = speedSupplier;
    this.xFunction = xFunction;
    this.yFunction = yFunction;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // joystick atan2 is positive = counterclockwise radians, 0 radians = +x axis
    // convert to positive = clockwise degrees, 0 degree = +y axis
    double joystickRotation = Math.toDegrees(Math.atan2(this.xFunction.get(), this.yFunction.get()));
    joystickRotation += joystickRotation < 0 ? 360 : 0;
    double robotRotation = drivebase.getAngle();
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
