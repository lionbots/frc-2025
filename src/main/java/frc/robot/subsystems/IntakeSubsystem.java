// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeSubsystem extends SubsystemBase {
  //Create instance variables for the motors
  private final SparkMax pivotMotor = new SparkMax(0, MotorType.kBrushless);
  //private final SparkMax intakeMotor = new SparkMax(1, MotorType.kBrushless);
  //Create instance variables for the encoders
  private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

  // Constructor to access the brake mode method
  public IntakeSubsystem() {
    setMotorIdleModes();
  }

  // Method for brake mode
  public void setMotorIdleModes() {
    SparkMaxConfig idleMode = new SparkMaxConfig();
    idleMode.idleMode(IdleMode.kBrake);

    pivotMotor.configure(idleMode, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //intakeMotor.configure(idleMode, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //Method for setting intake speed
  public void setIntakeSpeed(double intakeSpeed) {
    // intakeMotor.set(intakeSpeed);
    // intakeMotor.set(intakeSpeed);
  }

  //Method for setting pivot speed
  public void setPivotSpeed(double pivotSpeed) {
    pivotMotor.set(pivotSpeed);
  }
  
  //Method to get position of pivot
  public double getPivotPosition() {
    return pivotEncoder.getPosition();
  }
}