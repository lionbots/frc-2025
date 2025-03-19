// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class ClimberSubsystem extends SubsystemBase {

  private final SparkMax climberMotor = new SparkMax(0, MotorType.kBrushless);
  private final RelativeEncoder climberEncoder = climberMotor.getEncoder();

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    setMotorIdleModes();
    setCurrentLimit();
  }

  public void setMotorIdleModes() {
    SparkMaxConfig idleMode = new SparkMaxConfig();
    idleMode.idleMode(IdleMode.kBrake);
    climberMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setCurrentLimit() {
    SparkMaxConfig motorSpeed = new SparkMaxConfig();
    motorSpeed.smartCurrentLimit(40);
  }

  /*
   * Get climber position
   * @return the position of the climber motor according to the encoder
   */

  public double getPosition(){
    return climberEncoder.getPosition();
  }

  /*
   * Sets the speed of the motor. 
   * @param the desired speed for the motor
   */

  public void setSpeed(double speed){
    climberMotor.set(speed);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
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
