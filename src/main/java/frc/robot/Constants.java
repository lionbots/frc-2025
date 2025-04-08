// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

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
    public static final double kP = 0.011;
    public static final double kI = 0.001;
    public static final double kD = 0.000;

    public static final double tolerance = 2;
  }

  public static class IntakeConstants {
    public static int pivotMotorPort = 1;
    public static int intakeMotorPort = 3;
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
