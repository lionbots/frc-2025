// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DrivebaseSubsystem extends SubsystemBase {
    // front right motor, the type is brushless
    private SparkMax frMotor = new SparkMax(DriveConstants.frDeviceId, MotorType.kBrushless);
    // front left motor, the type is brushless
    private SparkMax flMotor = new SparkMax(DriveConstants.flDeviceId, MotorType.kBrushless);
    // back right motor, the type is brushless
    private SparkMax brMotor = new SparkMax(DriveConstants.brDeviceId, MotorType.kBrushless);
    // back left motor, the type is brushless
    private SparkMax blMotor = new SparkMax(DriveConstants.blDeviceId, MotorType.kBrushless);
    // DifferentialDrive with front left and front right motor.
    private DifferentialDrive d_drive = new DifferentialDrive(flMotor, frMotor);
    private RelativeEncoder frEncoder = frMotor.getEncoder();
    private RelativeEncoder flEncoder = flMotor.getEncoder();
    private AHRS navx2 = new AHRS(NavXComType.kMXP_SPI);

    public DrivebaseSubsystem() {
        // make back motors follow front motors, set idle braking, and limit current to 40 amps
        setFollow();
        setMotorIdleModes();
        setCurrentLimit();
    }

    /**
     * Get robot rotation
     * @return Z axis rotation in degrees, possibly above 360, from gyro
     */
    public double getAngle() {
        return navx2.getYaw();
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
    public void setFollow() {
        SparkMaxConfig leftFollow = new SparkMaxConfig();
        SparkMaxConfig rightFollow = new SparkMaxConfig();

        leftFollow.follow(flMotor);
        rightFollow.follow(frMotor);

        blMotor.configure(leftFollow, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        brMotor.configure(rightFollow, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * All the motor's idleModes would be set to brake
     */
    public void setMotorIdleModes() {
        SparkMaxConfig idleMode = new SparkMaxConfig();
        idleMode.idleMode(IdleMode.kBrake);

        frMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        brMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        brMotor.configure(idleMode, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Set to all motor's speed to 40 AMP max
     */
    public void setCurrentLimit() {
        SparkMaxConfig motorSpeed = new SparkMaxConfig();
        motorSpeed.smartCurrentLimit(40);

        frMotor.configure(motorSpeed, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flMotor.configure(motorSpeed, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        brMotor.configure(motorSpeed, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        brMotor.configure(motorSpeed, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Take parameter for speed and rotation for arcadeDrive and set motors accordingly
     * @param speed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param rotation The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is positive.
     */
    public void setDifferentialDrive(double speed, double rotation) {
        d_drive.arcadeDrive(speed, rotation);
    }
}
