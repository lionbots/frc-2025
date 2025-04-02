// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PIDConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeSubsystem extends SubsystemBase {
  //Create instance variables for the motors
  private final SparkMax pivotMotor = new SparkMax(IntakeConstants.pivotMotorPort, MotorType.kBrushless);
  private final SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeMotorPort, MotorType.kBrushless);
  //Create instance variables for the encoders
  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(IntakeConstants.encoderPort, 360, 0);
  // PID for the pivot
  private final PIDController pivotPID = new PIDController(PIDConstants.kIntakeP, PIDConstants.kIntakeI, PIDConstants.kIntakeP);

  // Constructor to access the brake mode method
  public IntakeSubsystem() {
    setMotorIdleModes();
    configurePID();
  }

  // Method for brake mode
  public void setMotorIdleModes() {
    SparkMaxConfig idleMode = new SparkMaxConfig();
    idleMode.idleMode(IdleMode.kBrake);

    pivotMotor.configure(idleMode, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(idleMode, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void configurePID() {
    pivotPID.enableContinuousInput(0, 360);
    pivotPID.setTolerance(PIDConstants.intakeTolerance);
  }

  //Method for setting intake speed
  public void setIntakeSpeed(double intakeSpeed) {
    intakeMotor.set(intakeSpeed);
  }

  public void setIntakePosition() {
    if(getPivotPosition() > 125 || getPivotPosition() < 25) {
      setPivotSpeed(-0.1);
    } 
    // else {
    //   setPivotSpeed(pivotPID.calculate(getPivotPosition(), IntakeConstants.pivotSetPoint));
    // }
  }

  //Method for setting pivot speed
  public void setPivotSpeed(double pivotSpeed) {
    pivotMotor.set(pivotSpeed);
  }
  
  //Method to get position of pivot
  public double getPivotPosition() {
    return pivotEncoder.get();
  }

  public boolean atPosition() {
    return pivotPID.atSetpoint();
  }

  public void periodic() {
    SmartDashboard.putNumber("Encoder", getPivotPosition());
    SmartDashboard.putNumber("PID", pivotPID.calculate(getPivotPosition(), IntakeConstants.pivotSetPoint));
  }
}