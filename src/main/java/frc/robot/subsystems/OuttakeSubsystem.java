package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OuttakeConstants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class OuttakeSubsystem extends SubsystemBase {
  //outer left motor on launcher
  private final SparkMax outerLMotor = new SparkMax(OuttakeConstants.outerLMotorPort, MotorType.kBrushless); 
  //outer right motor on launcher
  private final SparkMax outerRMotor = new SparkMax(OuttakeConstants.outerRMotorPort, MotorType.kBrushless);
  //inner front left motor on launcher
  private final SparkMax innerFLMotor = new SparkMax(OuttakeConstants.innerFLMotorPort, MotorType.kBrushless);
  //inner front right motor on launcher
  private final SparkMax innerFRMotor = new SparkMax(OuttakeConstants.innerFRMotorPort, MotorType.kBrushless);
  //inner back left motor on launcher
  private final SparkMax innerBLMotor = new SparkMax(OuttakeConstants.innerBLMotorPort, MotorType.kBrushless);
  //inner back right motor on launcher
  private final SparkMax innerBRMotor = new SparkMax(OuttakeConstants.innerBRMotorPort, MotorType.kBrushless);
  //motor for pivot
  private final SparkMax pivotMotor = new SparkMax(OuttakeConstants.pivotMotorPort, MotorType.kBrushless); 
  //encoder for the outer left motor on launcher
  private final RelativeEncoder outerLEncoder = outerLMotor.getEncoder();
  //encoder for the outer right motor on launcher
  private final RelativeEncoder outerREncoder = outerRMotor.getEncoder();
  //encoder for the pivot motor
  private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
  //beambreak for algea detection 
  private final DigitalInput beamBreak = new DigitalInput(OuttakeConstants.beamBreakPort);

  // only finna sim one motor cuz me lazy + no necessary to do all of them probably + actual thing prob don't have that many motors
  private final SparkMaxSim outerLMotorSim = new SparkMaxSim(outerLMotor, DCMotor.getNEO(1));
  private final FlywheelSim outerFlyWheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.00015, 1), DCMotor.getNEO(1), 0);

  private final SparkMaxSim innerFLMotorSim = new SparkMaxSim(innerFLMotor, DCMotor.getNEO(1));
  private final FlywheelSim innerFlywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.00015, 1), DCMotor.getNEO(1), 0);

  private final SparkMaxSim pivotMotorSim = new SparkMaxSim(pivotMotor, DCMotor.getNEO(1));
  private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), 1, 1, 0.559, 0, Math.PI, false, Math.PI / 2);

  public OuttakeSubsystem() {
    idleMotor();
  }

  @Override
  public void simulationPeriodic() {
    double vInVoltage = RoboRioSim.getVInVoltage();
    this.outerFlyWheelSim.setInput(this.outerLMotorSim.getAppliedOutput() * vInVoltage);
    this.outerFlyWheelSim.update(0.02);
    this.outerLMotorSim.iterate(this.outerFlyWheelSim.getAngularVelocityRPM(), vInVoltage, 0.02);

    this.innerFlywheelSim.setInput(this.innerFLMotorSim.getAppliedOutput() * vInVoltage);
    this.innerFlywheelSim.update(0.02);
    this.innerFLMotorSim.iterate(this.innerFlywheelSim.getAngularVelocityRPM(), vInVoltage, 0.02);

    this.pivotSim.setInput(this.pivotMotorSim.getAppliedOutput() * vInVoltage);
    this.pivotSim.update(0.02);
    this.pivotMotorSim.iterate(Units.radiansPerSecondToRotationsPerMinute(this.pivotSim.getVelocityRadPerSec()), vInVoltage, 0.02);

    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(this.outerFlyWheelSim.getCurrentDrawAmps() + this.innerFlywheelSim.getCurrentDrawAmps() + this.pivotSim.getCurrentDrawAmps()));

  }

  //set all the motors to brake mode
  public void idleMotor() {
    SparkMaxConfig idleMode = new SparkMaxConfig();
    idleMode.idleMode(IdleMode.kCoast);
    outerLMotor.configure(idleMode, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    outerRMotor.configure(idleMode, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    idleMode.idleMode(IdleMode.kBrake);
    pivotMotor.configure(idleMode, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    innerFLMotor.configure(idleMode, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    innerFRMotor.configure(idleMode, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    innerBLMotor.configure(idleMode, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    innerBRMotor.configure(idleMode, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  //set the speed to the outer motors
  public void setOuterSpeed(double outerSpeed) {
    outerLMotor.set(outerSpeed);
    outerRMotor.set(outerSpeed);
  }

  //set the speed to the inner motors
  public void setInnerSpeed(double innerSpeed) {
    innerFLMotor.set(innerSpeed);
    innerFRMotor.set(innerSpeed);
    innerBLMotor.set(innerSpeed);
    innerBRMotor.set(innerSpeed);
  }

  public void setPivotSpeed(double pivotSpeed){
    pivotMotor.set(pivotSpeed);
  }
  //get the position of the encoder values on the launcher
  public double outerLeftLauncherMotorPosition() {
    return outerLEncoder.getVelocity();
  }
  public double outerRightLauncherMotorPosition() {
    return outerREncoder.getVelocity();
  }
  public double pivotMotorPosition() {
    return pivotEncoder.getPosition();  
  }
  //get the state of the beambreak - detected = true and not detected = false
  public boolean getBeamBreak() {
    return !beamBreak.get();
  }
}