// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 5;
  }
  public static class DriveConstants {
    public static final int frDeviceId = 2;
    public static final int flDeviceId = 4;
    public static final int brDeviceId = 6;
    public static final int blDeviceId = 5;

    public static final int currentLimit = 40;
    public static final double slowSpeed = 0.5;

    // all these values are placeholders
    public static final int numMotors = 2;
    public static final int gearing = 8;
    public static final int momentIntertia = 3;
    public static final double massKg = 50;
    public static final double wheelRadiusMeters = 0.076;
    public static final double trackWidthMeters = 0.58;
    public static final Matrix<N7,N1> measurementStdDevs = null;
    public static final Pose2d simDefaultPose = new Pose2d(8.775, 4.025, new Rotation2d());

    // more placeholders cuz i couldnt get a robot to characterize
    public static final double ksVolts = 0.22;
    public static final double kvVoltsSecsPerMeter = 1.98;
    public static final double kaVoltSecsSquaredPerMeter = 0.2;
    public static final double kPDriveVel = 8.5;
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(trackWidthMeters);
  }
  public static class PIDConstants {
    public static final double kP = 0.011;
    public static final double kI = 0.001;
    public static final double kD = 0.000;

    public static final double tolerance = 2;
  }

  public static class IntakeConstants {
    public static int pivotMotorPort = 1;
    public static int intakeMotorPort = 3;
    public static final int encoderPort = 0;
    public static final double pivotSetpoint = 300;
    public static final double simPivotStartDeg = 99;
    public static final double pivotGearRatio = 3.0;
    public static double outtakeSpeed = -0.7;
  }

  public static class OuttakeConstants {
    public static int outerLMotorPort = 11;
    public static int outerRMotorPort = 9;
    public static int innerFLMotorPort = 10;
    public static int innerFRMotorPort = 8;
    public static int pivotMotorPort = 7;
    public static int beamBreakPort = 0;
  }
}
