// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.*;


public class IntakeSubsystem extends SubsystemBase {
  //Create instance variables for the motors
  private final SparkMax pivotMotor = new SparkMax(0, MotorType.kBrushless);
  private final SparkMax intakeRMotor = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax intakeLMotor = new SparkMax(2, null);
  //Create instance variables for the encoders
  private RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
  private RelativeEncoder intakeLMotorEncoder = intakeRMotor.getEncoder();
  private RelativeEncoder intakeRMotEncoder = intakeLMotor.getEncoder();

  // Constructor to access the brake mode method
  public IntakeSubsystem() {
    setMotorIdleModes();
  }

  // Method for brake mode
  public void setMotorIdleModes() {
    SparkMaxConfig idleMode = new SparkMaxConfig();
    idleMode.idleMode(IdleMode.kBrake);

    pivotMotor.configure(idleMode, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeRMotor.configure(idleMode, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeLMotor.configure(idleMode, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //Method for setting intake speed
  public void setIntakeSpeed(double intakeSpeed) {
    intakeRMotor.set(intakeSpeed);
    intakeLMotor.set(intakeSpeed);
  }

  //Method for setting pivot speed
  public void setPivotSpeed(double pivotSpeed) {
    pivotMotor.set(pivotSpeed);
  }

  //Method to get position of left intake motor
  public double getLIntakePosition() {
    return intakeLMotorEncoder.getPosition();

  }

  //Method to get position of right intake motor
  public double getRIntakePosition() {
    return intakeRMotEncoder.getPosition();
  }
  
  //Method to get position of pivot
  public double getPivotPosition() {
    return pivotEncoder.getPosition();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command intakeCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}