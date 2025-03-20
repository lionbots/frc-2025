// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivebaseSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class FieldCentricDriveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    // The X and Y values of a controler joystick
  private final Supplier<Double> xAxisFunction, yAxisFunction;
    // The forward and backward speed
  private final Supplier<Double> forwardSpeedFunction;
  // The drive base subsystem
  private final DrivebaseSubsystem drivebase;
  private final Supplier<Boolean> slowMode;


  // Creates a new ArcadeDriveCommand
  public FieldCentricDriveCommand(DrivebaseSubsystem drivebase, Supplier<Double> speedFunction, Supplier<Double> xAxisFunction, Supplier<Double> yAxisFunction, Supplier<Boolean> slowMode) {
    this.drivebase = drivebase;
    this.forwardSpeedFunction = speedFunction;
    this.xAxisFunction = xAxisFunction;
    this.yAxisFunction = yAxisFunction;
    this.slowMode = slowMode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Gets the forward speed backwards speed from the suppliers
    double forwardSpeed = forwardSpeedFunction.get();
    if (slowMode.get() == true) {
      forwardSpeed *= DriveConstants.slowSpeed;
    }

    // double rotation = rotationFunction.get();
    // joystick atan2 is positive = counterclockwise radians, 0 radians = +x axis
    // convert to positive = clockwise degrees, 0 degree = +y axis
    double xAxis = xAxisFunction.get();
    double yAxis = yAxisFunction.get() * -1;
    double joystickAngle = Math.toDegrees(Math.atan2(xAxis, yAxis));
    
    // Gets the rotation speed based on the the joystick heading and whether or not the robot is driving backwards
    double rotationSpeed = 0;
    if(xAxis > 0 || xAxis < 0 || yAxis > 0 || yAxis < 0) {
      rotationSpeed = drivebase.angleToRotation(joystickAngle, forwardSpeed < 0);
    }

    drivebase.setDifferentialDrive(forwardSpeed, rotationSpeed);
    
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
