// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PIDConstants;

public class DrivebaseSubsystem extends SubsystemBase {
    // front right motor, the type is brushless
    private final SparkMax frMotor = new SparkMax(DriveConstants.frDeviceId, MotorType.kBrushless);
    // front left motor, the type is brushless
    private final SparkMax flMotor = new SparkMax(DriveConstants.flDeviceId, MotorType.kBrushless);
    // back right motor, the type is brushless
    private final SparkMax brMotor = new SparkMax(DriveConstants.brDeviceId, MotorType.kBrushless);
    // back left motor, the type is brushless
    private final SparkMax blMotor = new SparkMax(DriveConstants.blDeviceId, MotorType.kBrushless);
    private final SparkMaxSim frMotorSim = new SparkMaxSim(frMotor, DCMotor.getNEO(1));
    private final SparkMaxSim flMotorSim = new SparkMaxSim(flMotor, DCMotor.getNEO(1));

    // DifferentialDrive with front left and front right motor.
    private final DifferentialDrive d_drive = new DifferentialDrive(flMotor, frMotor);
    private final RelativeEncoder frEncoder = frMotor.getEncoder();
    private final RelativeEncoder flEncoder = flMotor.getEncoder();
    private final DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(DCMotor.getNEO(DriveConstants.numMotors), DriveConstants.gearing, DriveConstants.momentIntertia, DriveConstants.massKg, DriveConstants.wheelRadiusMeters, DriveConstants.trackWidthMeters, DriveConstants.measurementStdDevs);

    private final AHRS navx2 = new AHRS(NavXComType.kUSB1);
    private final SimDouble yawSim = new SimDouble(SimDeviceDataJNI.getSimValueHandle(SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[2]"), "Yaw"));
    private final PIDController PID = new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD);

    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
        new Rotation2d(Math.toRadians(navx2.getYaw())),
        0,
        0,
        DriveConstants.simDefaultPose
    );
    public Field2d field = new Field2d();

    public DrivebaseSubsystem() {
        // make back motors follow front motors, set idle braking, and limit current to 40 amps
        // order of these functions matter apparently cuz ResetMode.kResetSafeParameters or something
        setFollow();
        setMotorIdleModes();
        configurePID();
        setInverted();
        setCurrentLimit();
        if (RobotBase.isSimulation()) {
            SmartDashboard.putData("Field", field);
            SmartDashboard.putData("Drive PID", this.PID);
            SmartDashboard.putData("navx2", navx2);
            // get reference to getNormalizedAngle() here cuz "this" has a different value inside the anonymous class
            DoubleSupplier bruh = this::getNormalizedAngle;
            // create a Sendable() with type "Gyro" to get that fancy gyro widget in simulation UI
            SmartDashboard.putData("Normalized angle", new Sendable() {
                @Override
                public void initSendable(SendableBuilder builder) {
                    builder.setSmartDashboardType("Gyro");
                    builder.addDoubleProperty("Value", bruh, null);
                }
            });
        }
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        double wheelCircumference = 2 * Math.PI * DriveConstants.wheelRadiusMeters;
        return new DifferentialDriveWheelSpeeds(flEncoder.getVelocity() / 60 * wheelCircumference, frEncoder.getVelocity() / 60 * wheelCircumference);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void voltageDrive(double leftVolts, double rightVolts) {
        flMotor.setVoltage(leftVolts);
        frMotor.setVoltage(rightVolts);
        d_drive.feed();
    }

    @Override
    public void periodic() {
        // RelativeEncoder supposed to return revolutions, need convert to meters
        // couldve used EncoderConfig.setPositionConversionFactor or something but im sure someones going to forget it exists
        double wheelCircumference = 2 * Math.PI * DriveConstants.wheelRadiusMeters;
        odometry.update(new Rotation2d(Math.toRadians(this.getNormalizedAngle())), this.getLeftPosition() * wheelCircumference, this.getRightPosition() * wheelCircumference);
        field.setRobotPose(getPose());
    }

    public void resetSimPos() {
        // for now wont reset angle or velocity or driveSim or encoder positions
        // those always cause problems like "mirroring" actions across simDefaultPose
        if (RobotBase.isSimulation()) {
            odometry.resetPose(DriveConstants.simDefaultPose);
        }
    }

    @Override
    public void simulationPeriodic() {
        driveSim.setInputs(flMotor.getAppliedOutput() * RoboRioSim.getVInVoltage(), frMotor.getAppliedOutput() * RoboRioSim.getVInVoltage());
        driveSim.update(0.02);

        // meters per second to revolutions per minute
        double conversionFactor = 60.0 / 2.0 / Math.PI / DriveConstants.wheelRadiusMeters;
        flMotorSim.iterate(driveSim.getLeftVelocityMetersPerSecond() * conversionFactor, RoboRioSim.getVInVoltage(), 0.02);
        frMotorSim.iterate(driveSim.getRightVelocityMetersPerSecond() * conversionFactor, RoboRioSim.getVInVoltage(), 0.02);

        double heading = driveSim.getHeading().getDegrees();
        yawSim.set(-heading);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(driveSim.getCurrentDrawAmps()));
    }

    // Makes the PID continuous at 0/360 and sets the tolerance to 2
    private void configurePID() {
        PID.enableContinuousInput(-180, 180);
        PID.setTolerance(PIDConstants.tolerance);
    }

    /**
     * Get front left wheel position
     * @return Number of rotations according to left encoder
     */
    public double getLeftPosition() {
        return flEncoder.getPosition();
    }

    /**
     * Get front right wheel position
     * @return Number of rotations according to right encoder
     */
    public double getRightPosition() {
        return frEncoder.getPosition();
    }

    /**
     * Make right and left back motors follow corresponding front motors
     */
    private void setFollow() {
        SparkMaxConfig leftFollow = new SparkMaxConfig();
        SparkMaxConfig rightFollow = new SparkMaxConfig();

        leftFollow.follow(flMotor);
        rightFollow.follow(frMotor);

        blMotor.configure(leftFollow, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        brMotor.configure(rightFollow, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setInverted() {
        SparkMaxConfig inverted = new SparkMaxConfig();
        inverted.inverted(true);
        frMotor.configure(inverted, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        brMotor.configure(inverted, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * All the motor's idleModes would be set to brake
     */
    private void setMotorIdleModes() {
        SparkMaxConfig idleMode = new SparkMaxConfig();
        idleMode.idleMode(IdleMode.kBrake);

        frMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        brMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        blMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Set to all motor's speed to 40 AMP max
     */
    private void setCurrentLimit() {
        SparkMaxConfig motorSpeed = new SparkMaxConfig();
        motorSpeed.smartCurrentLimit(DriveConstants.currentLimit);

        frMotor.configure(motorSpeed, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flMotor.configure(motorSpeed, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        brMotor.configure(motorSpeed, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        blMotor.configure(motorSpeed, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Take parameter for speed and rotation for arcadeDrive and set motors accordingly
     * @param speed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param rotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is positive.
     */
    public void setDifferentialDrive(double speed, double rotation) {
        d_drive.arcadeDrive(speed, rotation);
    }

    /**
     * Yaw according to navx2. -180 to 180 degrees, positive is clockwise
     * @return Yaw in degrees
     */
    public double getGyroAngle() {
        return navx2.getYaw();
    }

    public double getNormalizedAngle() {
        return getNormalizedAngle(false);
    }

    /**
     * Get yaw according to navx2.getAngle(), flip (add 180 degrees) if backward, normalize to 0 to 360, convert to -180 to 180
     * @param backwards
     * @return Normalized yaw
     */
    public double getNormalizedAngle(boolean backwards) {
        double gyroscopeAngle = -navx2.getAngle();
        gyroscopeAngle = ((gyroscopeAngle % 360) + 360) % 360;
        return gyroscopeAngle > 180 ? gyroscopeAngle - 360 : gyroscopeAngle;
    }

    // Returns an amount of motor effort/speed to turn based on the distance between the robot heading and a target point (0 - 180/-180°) using the PID
    public double angleToRotation(double target, boolean backwards) {
        return PID.calculate(getNormalizedAngle(backwards), target);
    }
}
