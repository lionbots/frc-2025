// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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
import frc.robot.SendableDouble;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class IntakeSubsystem extends SubsystemBase implements IMagicRotSubsystem {
    
    // Create instance variables for the motors
    private final SparkMax pivotMotor = new SparkMax(IntakeConstants.pivotMotorPort, MotorType.kBrushless);
    private final SparkMax intakeMotor = new SparkMax(IntakeConstants.intakeMotorPort, MotorType.kBrushless);
    private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(IntakeConstants.encoderPort, 1, 0);
    
    //Create instance variable for the motor simulation
    private final SparkMaxSim pivotMotorSim = new SparkMaxSim(pivotMotor, DCMotor.getNEO(1));
    private final SparkMaxSim intakeMotorSim = new SparkMaxSim(intakeMotor, DCMotor.getNEO(1));
    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), 100, SingleJointedArmSim.estimateMOI(0.2794, 5), 0.2794, 0, 2 * Math.PI, false, Math.toRadians(IntakeConstants.simPivotStartDeg));
    private final FlywheelSim intakeFlywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 1, 4), DCMotor.getNeo550(1));
    
    private MechanismLigament2d armLigament = null;
    private final PIDController pivotPid = new PIDController(0.005, 0, 0);
    private Double setpoint = null;
    
    // intake pivot is controlled by a motor attached to a small gear which is attached to large gear. small gear has 16 teeth, large has 48 so 3:1 input:output rotations
    // encoder is attached to small gear, so encoder reports three rotations for every intake pivot rotation
    // for magic align need convert encoder rotations to pivot rotations
    private SendableDouble encoderOffset = new SendableDouble(0, "intake pivot encoder offset");
    private SendableDouble numRotations = new SendableDouble(0);
    private double prevPivotPosition = RobotBase.isSimulation() ? (IntakeConstants.simPivotStartDeg - 90) * IntakeConstants.pivotGearRatio : pivotEncoder.get
    ();
    
    // intake pivot minimum negative velocity
    private SendableDouble negPivotVelocityLimit = new SendableDouble(-0.1, "negative pivot velocity limit");
    // intake pivot maximum positive velocity
    private SendableDouble posPivotVelocityLimit = new SendableDouble(0.1, "positive pivot velocity limit");
    private SendableDouble minPivotRot = new SendableDouble(-90, "minimum intake pivot rotation");
    private SendableDouble maxPivotRot = new SendableDouble(0, "maximum intake pivot rotation");
    
    // Constructor to access the brake mode method
    public IntakeSubsystem() {
        setMotorIdleModes();  
        SmartDashboard.putData("intake PID", pivotPid);
        this.pivotPid.enableContinuousInput(0, 360);
    }
    
    @Override
    public void simulationPeriodic() {
        double vInVoltage = RoboRioSim.getVInVoltage();
        
        this.pivotSim.setInput(pivotMotorSim.getAppliedOutput() * vInVoltage);
        this.pivotSim.update(0.02);
        // i have no clue how to simulate the gears and chain, so multiply arm velocity by gear ratio to get motor velocity
        this.pivotMotorSim.iterate(Units.radiansPerSecondToRotationsPerMinute(this.pivotSim.getVelocityRadPerSec() * IntakeConstants.pivotGearRatio), vInVoltage, 0.02);
        
        double pivotAngle = this.pivotSim.getAngleRads();
        if (MathUtil.isNear(2 * Math.PI, pivotAngle, 0.01) && this.pivotMotorSim.getAppliedOutput() > 0) {
            this.pivotSim.setState(0, this.pivotSim.getVelocityRadPerSec());
        }
        if (MathUtil.isNear(0, pivotAngle, 0.01) && this.pivotMotorSim.getAppliedOutput() < 0) {
            this.pivotSim.setState(2 * Math.PI, this.pivotSim.getVelocityRadPerSec());
        }
        
        double armAngle = Math.toDegrees(pivotAngle) - 90;
        // this.pivotEncoderSim.set(MathUtil.inputModulus(armAngle * IntakeConstants.pivotGearRatio + this.encoderOffset.getEncoderOffset(), 0, 360));
        
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
    
    // Method for setting intake speed
    public void setIntakeSpeed(double intakeSpeed) {
        intakeMotor.set(intakeSpeed);
    }
    
    // Method for setting pivot speed if 
    public void setPivotSpeed(double pivotSpeed) {
        double clampedPivotSpeed = MathUtil.clamp(pivotSpeed, negPivotVelocityLimit.getThing(), posPivotVelocityLimit.getThing());
        SmartDashboard.putNumber("intake pivot speed", pivotSpeed);
        SmartDashboard.putNumber("intake pivot clamped speed", clampedPivotSpeed);
        pivotMotor.set(clampedPivotSpeed);
    }
    
    // Method to get position of pivot
    private double getRawPivotPosition() {
        return RobotBase.isReal() ? this.pivotEncoder.get() : this.pivotMotorSim.getPosition();
    }
    
    // gets pivot position in degrees, compensating for gear ratio and encoder offset. can be <0 and >360
    private double getDiscontinuousPivotPosition() {
        // encoder rotation:intake pivot rotation = 3:1 so calculate accumulated rotation and divide by three
        double pivotPos = this.getRawPivotPosition() - this.encoderOffset.getThing();
        if (MathUtil.isNear(pivotPos, 360.0, 0.1)) {
            pivotPos = 0;
        }
        return (this.getRawPivotPosition() * 360 + pivotPos) / IntakeConstants.pivotGearRatio;
    }
    
    // gets pivot position with range [0, 360], compensating for gear ratio and encoder offset
    public double getPivotPosition() {
        return MathUtil.inputModulus(this.getDiscontinuousPivotPosition(), 0, 360);
    }
    
    /**
     * Checks if the intake pivot will remain in its rotation limit
     * @param speed Attempted speed of the intake pivot motor
     * @return Whether the intake pivot motor will remain within its rotation limits
     */
    public boolean pivotWithinBounds(double speed) {
        double pivotPos = this.getDiscontinuousPivotPosition();
        return (speed < 0 && pivotPos >= this.minPivotRot.getThing()) || (speed > 0 && pivotPos <= this.maxPivotRot.getThing()) || speed == 0;
    }

    public void periodic() {
        double rawPivotPosition = this.getRawPivotPosition();
        if (rawPivotPosition != this.prevPivotPosition) {
            if (this.prevPivotPosition > 340 && this.prevPivotPosition < 360 &&
            rawPivotPosition > 0 && rawPivotPosition < 20) {
                this.numRotations.setThing(this.numRotations.getThing() + 1);
            }
            if (this.prevPivotPosition > 0 && this.prevPivotPosition < 20 && rawPivotPosition > 340 && rawPivotPosition < 360) {
                this.numRotations.setThing(this.numRotations.getThing() - 1);
            }
        }
        
        SmartDashboard.putNumber("intake pivot raw rotation", rawPivotPosition);
        SmartDashboard.putNumber("intake pivot previous rotation", this.prevPivotPosition);
        SmartDashboard.putNumber("intake pivot offset rotation", this.getRawPivotPosition() - this.encoderOffset.getThing());
        SmartDashboard.putNumber("intake discontinuous rotation", this.getDiscontinuousPivotPosition());
        SmartDashboard.putNumber("intake true rotation", this.getPivotPosition());
        this.prevPivotPosition = rawPivotPosition;
        
        if (this.setpoint != null) {
            double calculation = this.pivotPid.calculate(this.getPivotPosition(), this.setpoint);
            SmartDashboard.putNumber("intake pid calculation", calculation);
            this.setPivotSpeed(calculation);
        }
        
        // if intake pivot motor is attempting to go past limits, stop it
        if (!this.pivotWithinBounds(this.pivotMotor.get())) {
            this.setPivotSpeed(0);
        }
    }
    
    public void setSetpoint(Double pos) {
        this.setpoint = pos;
    }
    
    public Double getSetpoint() {
        return this.setpoint;
    }
    
    public boolean atSetPoint() {
        return this.pivotPid.atSetpoint();
    }
}