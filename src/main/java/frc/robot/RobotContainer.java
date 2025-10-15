// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
* This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
*/
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DrivebaseSubsystem drivebase = new DrivebaseSubsystem();
    
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.driverControllerPort);    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // left trigger axis is definitely not the climber axis i just need a placeholder
        drivebase.setDefaultCommand(new FieldCentricDriveCommand(drivebase, driverController::getRightTriggerAxis, () -> driverController.getLeftTriggerAxis() * -1, driverController::getLeftX, () -> driverController.getLeftY() * -1, () -> driverController.rightBumper().getAsBoolean()));
        // Configure the trigger bindings
        configureBindings();
    }

    /**
    * Use this method to define your trigger->command mappings. Triggers can be created via the
    * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
    * predicate, or via the named factories in {@link
    * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
    * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
    * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
    * joysticks}.
    */
    private void configureBindings() {
        driverController.x().whileTrue(
            drivebase.resetEncoders()
            .andThen(drivebase.routine.quasistatic(SysIdRoutine.Direction.kForward))
            .andThen(drivebase.routine.quasistatic(SysIdRoutine.Direction.kReverse))
            .andThen(drivebase.routine.dynamic(SysIdRoutine.Direction.kReverse))
            .andThen(drivebase.routine.dynamic(SysIdRoutine.Direction.kReverse))
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return null;
    }
}
