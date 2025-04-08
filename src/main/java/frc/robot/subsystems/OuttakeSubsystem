package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OuttakeConstants;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class OuttakeSubsystem extends SubsystemBase {
  //outer left motor on launcher
  private final SparkMax outerLMotor = new SparkMax(OuttakeConstants.outerLMotorPort, MotorType.kBrushless); 
  //outer right motor on launcher
  private final SparkMax outerRMotor = new SparkMax(OuttakeConstants.outerRMotorPort, MotorType.kBrushless);
  //inner front left motor on launcher
  private final SparkMax innerFLMotor = new SparkMax (OuttakeConstants.innerFLMotorPort, MotorType.kBrushless);
  //inner front right motor on launcher
  private final SparkMax innerFRMotor = new SparkMax (OuttakeConstants.innerFRMotorPort, MotorType.kBrushless);
  //motor for pivot
  private final SparkMax pivotMotor = new SparkMax(OuttakeConstants.pivotMotorPort, MotorType.kBrushless); 
  // //encoder for the outer left motor on launcher
  // private final RelativeEncoder outerLEncoder = outerLMotor.getEncoder();
  // //encoder for the outer right motor on launcher
  // private final RelativeEncoder outerREncoder = outerRMotor.getEncoder();
  // //encoder for the pivot motor
  // private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
  // //beambreak for algea detection 
  // private final DigitalInput beamBreak = new DigitalInput(0);


    public OuttakeSubsystem() {
      idleMotor();
    }
    //set all the motors to brake mode
    public void idleMotor() {
      SparkMaxConfig idleMode = new SparkMaxConfig();
      idleMode.idleMode(IdleMode.kCoast);
      outerLMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      outerRMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      idleMode.idleMode(IdleMode.kBrake);
      pivotMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      innerFLMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      innerFRMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    //set the speed to the outer motors
    public void setOuttakeSpeed(double speed) {
      outerLMotor.set(speed * -1);
      outerRMotor.set(speed);
      innerFLMotor.set(speed * 0.8);
      innerFRMotor.set(speed * 0.8);
    }

    public void setPivotSpeed(double pivotSpeed){
      pivotMotor.set(pivotSpeed);
    }
    // //get the position of the encoder values on the launcher
    // public double outerLeftLauncherMotorPosition() {
    //   return outerLEncoder.getVelocity();
    // }
    // public double outerRightLauncherMotorPosition() {
    //   return outerREncoder.getVelocity();
    // }
    // public double pivotMotorPosition() {
    //   return pivotEncoder.getPosition();  
    // }
    // //get the state of the beambreak - detected = true and not detected = false
    // public boolean getBeamBreak() {
    //   return !beamBreak.get();
    // }
}