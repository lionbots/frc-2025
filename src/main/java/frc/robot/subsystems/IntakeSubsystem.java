// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.IMagicRotSubsystem;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeSubsystem extends SubsystemBase implements IMagicRotSubsystem {

  //Create instance variables for the motors
  private final SparkMax pivotMotor = new SparkMax(IntakeConstants.pivotMotorPort, MotorType.kBrushless);
  private final SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeMotorPort, MotorType.kBrushless);
  
  //Create instance variables for the encoders
  private final AbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();
  // have to simulate encoder cuz AbsoluteEncoder constantly gives 0 in simulation
  private final SparkAbsoluteEncoderSim pivotEncoderSim = new SparkAbsoluteEncoderSim(pivotMotor);

  //Create instance variable for the motor simulation
  private final SparkMaxSim pivotMotorSim = new SparkMaxSim(pivotMotor, DCMotor.getNEO(1));
  private final SparkMaxSim intakeMotorSim = new SparkMaxSim(intakeMotor, DCMotor.getNEO(1));
  private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), 100, SingleJointedArmSim.estimateMOI(0.2794, 5), 0.2794, 0, 2 * Math.PI, true, Math.PI / 2);
  private final FlywheelSim intakeFlywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 1, 4), DCMotor.getNeo550(1));

  private MechanismLigament2d armLigament = null;

  // Constructor to access the brake mode method
  public IntakeSubsystem() {
    setMotorIdleModes();
  }

  @Override
  public void simulationPeriodic() {
    double vInVoltage = RoboRioSim.getVInVoltage();

    this.pivotSim.setInput(pivotMotorSim.getAppliedOutput() * vInVoltage);
    this.pivotSim.update(0.02);
    this.pivotMotorSim.iterate(Units.radiansPerSecondToRotationsPerMinute(this.pivotSim.getVelocityRadPerSec()), vInVoltage, 0.02);
    this.pivotEncoderSim.setPosition(this.pivotSim.getAngleRads() / 2 / Math.PI);
    if (this.armLigament != null) {
      // anglE RElatIve To iTs pArent
      this.armLigament.setAngle(Math.toDegrees(this.pivotSim.getAngleRads()) - 90);
    }

    this.intakeFlywheelSim.setInput(intakeMotorSim.getAppliedOutput() * vInVoltage);
    this.intakeFlywheelSim.update(0.02);
    this.intakeMotorSim.iterate(this.intakeFlywheelSim.getAngularVelocityRPM(), vInVoltage, 0.02);

    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(this.pivotSim.getCurrentDrawAmps()));
  }

  public IntakeSubsystem setBaseLigament(MechanismLigament2d baseLigament) {
    this.armLigament = baseLigament.append(new MechanismLigament2d("intake", 1, 90, 6, new Color8Bit(255, 0, 0)));
    return this;
  }

  public double getEncoder() {
    return pivotEncoder.getPosition();
  }

  // Method for brake mode
  public void setMotorIdleModes() {
    SparkMaxConfig idleMode = new SparkMaxConfig();
    idleMode.idleMode(IdleMode.kBrake);

    pivotMotor.configure(idleMode, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    intakeMotor.configure(idleMode, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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
    return RobotBase.isSimulation() ? pivotEncoderSim.getPosition() : pivotEncoder.getPosition();
  }

  public void periodic() {
    SmartDashboard.putNumber("Encoder", getEncoder());
  }
}