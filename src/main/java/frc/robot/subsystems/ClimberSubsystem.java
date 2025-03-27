// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.IMagicRotSubsystem;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


public class ClimberSubsystem extends SubsystemBase implements IMagicRotSubsystem {
  private final SparkMax climberMotor = new SparkMax(ClimberConstants.climberMotorPort, MotorType.kBrushless);
  private final DCMotor dcmotor = DCMotor.getNEO(1);
  private final SparkMaxSim climberMotorSim = new SparkMaxSim(climberMotor, dcmotor);
  
  // made up values, me need cad to be done
  private final SingleJointedArmSim armSim = new SingleJointedArmSim(dcmotor, 100, SingleJointedArmSim.estimateMOI(0.254, 5), 0.254, 0, Math.PI, true, 0);
  private final RelativeEncoder climberEncoder = climberMotor.getEncoder();

  private final PIDController pid = new PIDController(1, 0, 0);
  private Double setpoint = 0.0 / 0.0;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    setMotorIdleModes();
    setCurrentLimit();
    SmartDashboard.putData("climber PID", pid);
  }

  @Override
  public void simulationPeriodic() {
    this.armSim.setInput(climberMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
    this.armSim.update(0.02);
    this.climberMotorSim.iterate(Units.radiansPerSecondToRotationsPerMinute(this.armSim.getVelocityRadPerSec()), RoboRioSim.getVInVoltage(), 0.02);
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(this.armSim.getCurrentDrawAmps()));
  }

  public void setMotorIdleModes() {
    SparkMaxConfig idleMode = new SparkMaxConfig();
    idleMode.idleMode(IdleMode.kBrake);
    climberMotor.configure(idleMode, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setCurrentLimit() {
    SparkMaxConfig motorSpeed = new SparkMaxConfig();
    motorSpeed.smartCurrentLimit(40);
  }

  /*
   * Get climber position
   * @return the position of the climber motor according to the encoder
   */
  public double getPivotPosition(){
    return climberEncoder.getPosition();
  }

  /*
   * Sets the speed of the motor. 
   * @param the desired speed for the motor
   */
  public void setPivotSpeed(double speed){
    climberMotor.set(speed);
  }

  public void setSetpoint(double pos) {
    this.setpoint = pos;
  }

  public boolean atSetPoint() {
    System.out.println(this.pid.atSetpoint());
    return this.pid.atSetpoint();
  }

  @Override
  public void periodic() {
      if (!this.setpoint.isNaN()) {
        this.setPivotSpeed(this.pid.calculate(this.getPivotPosition(), this.setpoint));
      }
  }
}
