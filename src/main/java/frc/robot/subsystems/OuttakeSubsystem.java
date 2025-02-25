package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class OuttakeSubsystem extends SubsystemBase {
  //outer left motor on launcher
  private final SparkMax outerLMotor = new SparkMax(0, MotorType.kBrushless); 
  //outer right motor on launcher
  private final SparkMax outerRMotor = new SparkMax(0, MotorType.kBrushless);
  //inner left motor on launcher
  private final SparkMax innerLMotor = new SparkMax (0, MotorType.kBrushless);
  //inner right motor on launcher
  private final SparkMax innerRMotor = new SparkMax (0, MotorType.kBrushless);
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
      innerLMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      innerRMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      idleMode.idleMode(IdleMode.kBrake);
      pivotMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    //set the speed to the motors
    public void setOuttakeSpeed(double outtakeSpeed) {
      outerLMotor.set(outtakeSpeed);
      outerRMotor.set(outtakeSpeed);
      innerLMotor.set(outtakeSpeed);
      innerRMotor.set(outtakeSpeed);
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

    public Command outtakeMethodCommand() {
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
      public boolean outtakeCondition() {
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
