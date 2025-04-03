// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.IMagicRotSubsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PIDConstants;

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
  
  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(IntakeConstants.encoderPort, 360, 0);
  private final DutyCycleEncoderSim pivotEncoderSim = new DutyCycleEncoderSim(pivotEncoder);

  //Create instance variable for the motor simulation
  private final SparkMaxSim pivotMotorSim = new SparkMaxSim(pivotMotor, DCMotor.getNEO(1));
  private final SparkMaxSim intakeMotorSim = new SparkMaxSim(intakeMotor, DCMotor.getNEO(1));
  private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), 100, SingleJointedArmSim.estimateMOI(0.2794, 5), 0.2794, 0, 2 * Math.PI, false, Math.toRadians(IntakeConstants.simPivotStartDeg));
  private final FlywheelSim intakeFlywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 1, 4), DCMotor.getNeo550(1));

  private MechanismLigament2d armLigament = null;
  private final PIDController pivotPid = new PIDController(PIDConstants.kIntakeP, PIDConstants.kIntakeI, PIDConstants.kIntakeD);
  private Double setpoint = null;

  // intake pivot is controlled by a motor attached to a small gear which is attached to large gear. small gear has 16 teeth, large has 48 so 3:1 input:output rotations
  // encoder is attached to small gear, so encoder reports three rotations for every intake pivot rotation
  // for magic align need convert encoder rotations to pivot rotations
  // duty cycle encoder is continuous or whatever so have to count number of rotations and calcualte accumulated rotations
  // dont know why numRotations starts at -1, it just works kinda sorta maybe probably
  private int numRotations = 0;
  // previous encoder value to detect when 360 degrees turns to 0 degrees
  private double prevPivotPosition = RobotBase.isSimulation() ? (IntakeConstants.simPivotStartDeg - 90) * IntakeConstants.pivotGearRatio : pivotEncoder.get();

  class EncoderOffsetSendable implements Sendable {
    private double encoderOffset = 0;

    public double getEncoderOffset() {
      return this.encoderOffset;
    }

    public void setEncoderOffset(double newOffset) {
      this.encoderOffset = newOffset;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("encoder offset", this::getEncoderOffset, this::setEncoderOffset);
    }
  }
  private EncoderOffsetSendable encoderOffset = new EncoderOffsetSendable();

  // Constructor to access the brake mode method
  public IntakeSubsystem() {
    setMotorIdleModes();  
    SmartDashboard.putData("intake PID", pivotPid);
    SmartDashboard.putData("intake pivot encoder offset", encoderOffset);
    this.pivotPid.enableContinuousInput(0, 360);
    this.pivotEncoderSim.set(this.prevPivotPosition);
  }

  @Override
  public void simulationPeriodic() {
    double vInVoltage = RoboRioSim.getVInVoltage();

    this.pivotSim.setInput(pivotMotorSim.getAppliedOutput() * vInVoltage);
    this.pivotSim.update(0.02);
    this.pivotMotorSim.iterate(Units.radiansPerSecondToRotationsPerMinute(this.pivotSim.getVelocityRadPerSec()), vInVoltage, 0.02);

    double pivotAngle = this.pivotSim.getAngleRads();
    if (MathUtil.isNear(2 * Math.PI, pivotAngle, 0.01) && this.pivotMotorSim.getAppliedOutput() > 0) {
      this.pivotSim.setState(0, this.pivotSim.getVelocityRadPerSec());
    }
    if (MathUtil.isNear(0, pivotAngle, 0.01) && this.pivotMotorSim.getAppliedOutput() < 0) {
      this.pivotSim.setState(2 * Math.PI, this.pivotSim.getVelocityRadPerSec());
    }
  
    double armAngle = Math.toDegrees(pivotAngle) - 90;
    this.pivotEncoderSim.set(MathUtil.inputModulus(armAngle * IntakeConstants.pivotGearRatio + this.encoderOffset.getEncoderOffset(), 0, 360));

    if (this.armLigament != null) {
      // anglE RElatIve To iTs pArent
      this.armLigament.setAngle(armAngle);
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
  private double getRawPivotPosition() {
    return RobotBase.isSimulation() ? pivotEncoderSim.get() : pivotEncoder.get();
  }

  public double getPivotPosition() {
    // encoder rotation:intake pivot rotation = 3:1 so calculate accumulated rotation and divide by three
    double pivotPos = this.getRawPivotPosition() - this.encoderOffset.getEncoderOffset();
    if (MathUtil.isNear(pivotPos, 360.0, 0.1)) {
      pivotPos = 0;
    }
    return MathUtil.inputModulus((this.numRotations * 360 + pivotPos) / IntakeConstants.pivotGearRatio, 0, 360);
  }

  public void periodic() {
    // detect intake pivot 360 -> 0 degrees
    // if encoder was previously 340 or something and now its 20 then +1 rotation
    // if it was 20 and now its 340 then -1 rotation
    // 20 chosen because thats around how much the intake moves every time this method called
    double rawPivotPosition = this.getRawPivotPosition();
    if (rawPivotPosition != this.prevPivotPosition) {
      if (this.prevPivotPosition > 340 && this.prevPivotPosition < 360 && rawPivotPosition > 0 && rawPivotPosition < 20) {
        this.numRotations++;
      }
      if (this.prevPivotPosition > 0 && this.prevPivotPosition < 20 && rawPivotPosition > 340 && rawPivotPosition < 360) {
        this.numRotations--;
      }
    }
    SmartDashboard.putNumber("intake previous raw rotation", this.prevPivotPosition);
    SmartDashboard.putNumber("intake pivot num rotations", this.numRotations);
    SmartDashboard.putNumber("intake pivot raw rotation", rawPivotPosition);
    SmartDashboard.putNumber("intake pivot offset rotation", rawPivotPosition - this.encoderOffset.getEncoderOffset());
    SmartDashboard.putNumber("intake true rotation", this.getPivotPosition());
    this.prevPivotPosition = rawPivotPosition;

    if (this.setpoint != null) {
      double calculation = this.pivotPid.calculate(this.getPivotPosition(), this.setpoint);
      SmartDashboard.putNumber("intake pid calculation", this.pivotPid.calculate(this.getPivotPosition(), this.setpoint));
      this.setPivotSpeed(calculation);
    }
  }
  
  public void setSetpoint(Double pos) {
    this.setpoint = pos;
  }

  public boolean atSetPoint() {
    return this.pivotPid.atSetpoint();
  }
}