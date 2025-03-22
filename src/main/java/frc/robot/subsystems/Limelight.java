package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.networktables.*;


public class Limelight extends SubsystemBase {
    // Network table for the limelight
    private final NetworkTable limelightTable;
    // Position of the robot
    private static NetworkTableEntry robotPose;
     // Horizontal offset from crosshair to note
    private static NetworkTableEntry txEntry;
    // Whether or not a algae is detected (1 = true, 0 = false)
    private static NetworkTableEntry tvEntry;


    public Limelight() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        txEntry = limelightTable.getEntry("tx");
        tvEntry = limelightTable.getEntry("tv");
        robotPose = limelightTable.getEntry("botpose_orb");
    }

    // Returns the value of tx from the limelight
    public static double getTx() {
        return txEntry.getDouble(LimelightConstants.defaultTx);
    }
    
    // Returns the value of tv from the limelight
    public static int getTv() {
        return (int) tvEntry.getInteger(LimelightConstants.defaultTv);
    }
    
    // Retruns the position of the robot from the limelight
    public static double[] getRobotPose() {
        return robotPose.getDoubleArray(LimelightConstants.defaultRobotPose);
    }

}