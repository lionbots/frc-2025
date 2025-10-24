package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.geofence.GeofenceObject;
import frc.robot.subsystems.DrivebaseSubsystem;

public class GeofenceDriveCommand extends Command {
    private final DrivebaseSubsystem drivebase;
    private final Supplier<Double> rotationSupplier;
    private final Supplier<Double> speedSupplier;
    private final GeofenceObject[] geofenceObjects;
    
    // private final double maximumSpeed = 4;
    private final double robotRadius = 0.8;

    /**
     * Constructs a GeofenceDriveCommand
     * @param drivebase Drivebase subsystem
     * @param rotationSupplier Input relative rotation in radians. NaN if no rotation change inputted
     * @param speedSupplier Velocity from -1 to 1 with a magnitude of 1 meaning maximum speed
     * @param geofenceObjects Objects to avoid
     */
    public GeofenceDriveCommand(DrivebaseSubsystem drivebase, Supplier<Double> rotationSupplier, Supplier<Double> speedSupplier, GeofenceObject[] geofenceObjects) {
        this.drivebase = drivebase;
        this.rotationSupplier = rotationSupplier;
        this.speedSupplier = speedSupplier;
        this.geofenceObjects = geofenceObjects;
        this.addRequirements(drivebase);
    }

    @Override
    public void execute() {
        // convert rotation and speed into XY, modify velocity, convert back into rotation and speed
        Pose2d robotPose = this.drivebase.getPose();
        double suppliedRotation = this.rotationSupplier.get();
        double suppliedSpeed = this.speedSupplier.get();
        if (Math.abs(suppliedSpeed) < 0.01) {
            suppliedSpeed = 0;
        }
        Rotation2d inputRotation = Double.isNaN(suppliedRotation) ? robotPose.getRotation() : new Rotation2d(suppliedRotation);
        Translation2d inputMotion = new Translation2d(suppliedSpeed, inputRotation);
        SmartDashboard.putString("drive inputs", "speed: " + suppliedSpeed + " rotation: " + inputRotation.getDegrees());
        SmartDashboard.putString("input cartesian velocity", inputMotion.getX() + " " + inputMotion.getY());
        if (suppliedSpeed != 0) {
            SmartDashboard.putString("input polar velocity", "speed: " + inputMotion.getNorm() + " rotation: " + inputMotion.getAngle().getDegrees() + " source rotation: " + inputRotation);
        }
        for (GeofenceObject object : geofenceObjects) {
            inputMotion = object.modifyMotion(inputMotion, robotPose.getTranslation(), this.robotRadius);
        }
        // if speed is 0, inputMotion.getAngle() will fail
        // NaN is for a good reason i swear i forgor why
        // the Math.signum thing is a cheap hack and will probably cause problems
        drivebase.setDifferentialDrive(inputMotion.getNorm() * Math.signum(suppliedSpeed), suppliedSpeed == 0 && Double.isNaN(suppliedRotation) ? 0: drivebase.angleToRotation(suppliedSpeed == 0 ? suppliedRotation : inputMotion.getAngle().getDegrees(), suppliedSpeed < 0));
    }
}
