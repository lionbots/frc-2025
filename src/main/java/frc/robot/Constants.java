// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  }
  public static class PIDConstants {
    public static final double kP = 0.008;
    public static final double kI = 0.001;
    public static final double kD = 0;

    public static final double tolerance = 2;
  }

  public static class IntakeConstants {
    public static int pivotMotorPort = 1;
    public static int intakeMotorPort = 3;
    public static double outtakeSpeed = -1;
  }

  public static class OuttakeConstants {
    public static int outerLMotorPort = 0;
    public static int outerRMotorPort = 0;
    public static int innerFLMotorPort = 0;
    public static int innerFRMotorPort = 0;
    public static int innerBLMotorPort = 0;
    public static int innerBRMotorPort = 0;
    public static int pivotMotorPort = 0;
    public static int beamBreakPort = 0;
  }

  public static class ClimberConstants {
    // placeholder values
    public static int climberMotorPort = 7;
  }
}
