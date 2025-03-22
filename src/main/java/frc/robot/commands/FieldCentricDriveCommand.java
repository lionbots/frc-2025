// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivebaseSubsystem;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class FieldCentricDriveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    // The X and Y values of a controler joystick
  private final Supplier<Double> xAxisFunction, yAxisFunction;
    // The forward and backward speed
  private final Supplier<Double> forwardSpeedFunction, backwardSpeedFunction;
  // The drive base subsystem
  private final DrivebaseSubsystem drivebase;
  private final Supplier<Boolean> slowMode;
  // Encoders
  private final DrivebaseSubsystem frEncoder;
  private final DrivebaseSubsystem flEncoder;
  // Track Width Meters
  private final DifferentialDriveKinematics trackWidthMeters = new DifferentialDriveKinematics(null);
  // Get gyro angle
  private final double degree = new Rotation2d().getDegrees();
  private final Rotation2d gyroAngle = new Rotation2d().fromDegrees(degree);
  // Pose Estimator
  private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(
    trackWidthMeters, 
    gyroAngle, 
    flEncoder.getPosition(), 
    frEncoder.getPosition(), 
    new Pose2d()
    );


  // Creates a new ArcadeDriveCommand
  public FieldCentricDriveCommand(DrivebaseSubsystem drivebase, Supplier<Double> forwardSpeedFunction, Supplier<Double> backwardSpeedFunction, Supplier<Double> xAxisFunction, Supplier<Double> yAxisFunction, Supplier<Boolean> slowMode) {
    this.drivebase = drivebase;
    this.forwardSpeedFunction = forwardSpeedFunction;
    this.backwardSpeedFunction = backwardSpeedFunction;
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
    double backwardSpeed = backwardSpeedFunction.get();
    if (slowMode.get() == true) {
      forwardSpeed *= DriveConstants.slowSpeed;
      backwardSpeed *= DriveConstants.slowSpeed;
    }

    // double rotation = rotationFunction.get();
    // joystick atan2 is positive = counterclockwise radians, 0 radians = +x axis
    // convert to positive = clockwise degrees, 0 degree = +y axis
    double xAxis = xAxisFunction.get();
    double yAxis = yAxisFunction.get() * -1;
    double joystickAngle = Math.toDegrees(Math.atan2(xAxis, yAxis));
    
    // Gets the rotation speed based on the the joystick heading and whether or not the robotis driving backwards
    double rotationSpeed = 0;
    if(xAxis > 0 || xAxis < 0 || yAxis > 0 || yAxis < 0) {
      rotationSpeed = drivebase.angleToRotation(joystickAngle, backwardSpeed > 0);
    }

    // Drives the robot either forward or backwards back on the whether or not the left trigger is pressed
    if(backwardSpeed < 0) {
      drivebase.setDifferentialDrive(backwardSpeed, rotationSpeed);
    } else {
      drivebase.setDifferentialDrive(forwardSpeed, rotationSpeed);
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
