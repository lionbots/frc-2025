package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class OuttakeSubsystem extends SubsystemBase {
  //outer left motor on launcher
  private final SparkMax outerLMotor = new SparkMax(0, MotorType.kBrushless); 
  //outer right motor on launcher
  private final SparkMax outerRMotor = new SparkMax(0, MotorType.kBrushless);
  //inner front left motor on launcher
  private final SparkMax innerFLMotor = new SparkMax (0, MotorType.kBrushless);
  //inner front right motor on launcher
  private final SparkMax innerFRMotor = new SparkMax (0, MotorType.kBrushless);
  //inner back left motor on launcher
  private final SparkMax innerBLMotor = new SparkMax (0, MotorType.kBrushless);
  //inner back right motor on launcher
  private final SparkMax innerBRMotor = new SparkMax (0, MotorType.kBrushless);
  //motor for pivot
  private final SparkMax pivotMotor = new SparkMax(0,MotorType.kBrushless); 
  //encoder for the outer left motor on launcher
  private final RelativeEncoder outerLEncoder = outerLMotor.getEncoder();
  //encoder for the outer right motor on launcher
  private final RelativeEncoder outerREncoder = outerRMotor.getEncoder();
  //encoder for the pivot motor
  private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder(); 

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
      innerBLMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      innerBRMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
}
