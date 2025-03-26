// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
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
  private final AbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();

  //Create instance variable for the motor simulation
  private final SparkMaxSim pivotMotorSim = new SparkMaxSim(pivotMotor, DCMotor.getNEO(1));
  private final SparkMaxSim intakeMotorSim = new SparkMaxSim(intakeMotor, DCMotor.getNEO(1));
  private final SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getNEO(1), 1, 1, 0.2794, 0, Math.PI, false, 0);
  private final SparkRelativeEncoderSim pivotEncoderSim = pivotMotorSim.getRelativeEncoderSim();

  // Constructor to access the brake mode method
  public IntakeSubsystem() {
    setMotorIdleModes();
  }

  @Override
  public void simulationPeriodic() {
    
    this.armSim.setInput(pivotMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    this.armSim.update(0.02);

    // simulated motor has velocity without getAppliedOutput()
    if (pivotMotorSim.getAppliedOutput() == 0) {
      this.armSim.setState(this.armSim.getAngleRads(), 0);
      this.pivotMotorSim.iterate(0, RoboRioSim.getVInVoltage(), 0.02);
    } else {
      this.pivotMotorSim.iterate(Units.radiansPerSecondToRotationsPerMinute(this.armSim.getVelocityRadPerSec()), RoboRioSim.getVInVoltage(), 0.02);
    }
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(this.armSim.getCurrentDrawAmps()));


  }

  public double getEncoder() {
    return pivotEncoder.getPosition();
  }

  // Method for brake mode
  public void setMotorIdleModes() {
    SparkMaxConfig idleMode = new SparkMaxConfig();
    idleMode.idleMode(IdleMode.kBrake);

    pivotMotor.configure(idleMode, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(idleMode, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  //Method for setting intake speed
  public void setIntakeSpeed(double intakeSpeed) {
    intakeMotor.set(intakeSpeed);
  }

  //Method for setting pivot speed
  public void setPivotSpeed(double pivotSpeed) {
    pivotMotor.set(pivotSpeed);
  }
  
  //Method to get position of pivot
  public double getPivotPosition() {
    return pivotEncoder.getPosition();
  }

  /*
   * Get climber position
   * @return the position of the climber motor according to the encoder
   */

  public double getPosition(){
    return RobotBase.isSimulation() ? pivotEncoderSim.getPosition() : pivotEncoderSim.getPosition();
  }
  /*
   * Sets the speed of the motor. 
   * @param the desired speed for the motor
   */

  public void periodic() {
    SmartDashboard.putNumber("Encoder", getEncoder());
  }
}