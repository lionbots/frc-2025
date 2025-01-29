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
  //left motor on launcher
  private final SparkMax leftLauncherMotor = new SparkMax(0, MotorType.kBrushless); 
  //right motor on launcher
  private final SparkMax rightLauncherMotor = new SparkMax(0, MotorType.kBrushless);
  //motor for pivot
  private final SparkMax pivotMotor = new SparkMax(0,MotorType.kBrushless); 
  //encoder for the left motor on launcher
  private final RelativeEncoder leftlauncherEncoder = leftLauncherMotor.getEncoder();
  //encoder for the right motor on launcher
  private final RelativeEncoder rightlauncherEncoder = rightLauncherMotor.getEncoder();
  //encoder for the pivot motor
  private final RelativeEncoder pivotEncoder = pivotMotor.getEncoder(); 

    public OuttakeSubsystem() {
      idleMotor();
    }
    //set all the motors to brake mode
    public void idleMotor() {
      SparkMaxConfig idleMode = new SparkMaxConfig();
      idleMode.idleMode(IdleMode.kBrake);
      leftLauncherMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      rightLauncherMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      pivotMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setOuttakeSpeed(double outtakeSpeed) {
      leftLauncherMotor.set(outtakeSpeed);
      rightLauncherMotor.set(outtakeSpeed);
    }

    public void setPivotSpeed(double pivotSpeed){
      pivotMotor.set(pivotSpeed);
    }
    public double leftLauncherMotorPosition() {
      return leftlauncherEncoder.getPosition();
    }
    public double rightLauncherMotorPosition() {
      return rightlauncherEncoder.getPosition();
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
