// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.OuttakeSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final OuttakeSubsystem outtake = new OuttakeSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final DrivebaseSubsystem drivebase = new DrivebaseSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.driverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.operatorControllerPort);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    intake.setDefaultCommand(new IntakePivotCommand(intake, () -> operatorController.getLeftY() * -1));
    outtake.setDefaultCommand(new OuttakePivotCommand(outtake, () -> operatorController.getRightY() * -1));
    drivebase.setDefaultCommand(new FieldCentricDriveCommand(drivebase, () -> driverController.getRightTriggerAxis(), () -> driverController.getLeftTriggerAxis() * -1, () -> driverController.getLeftX(), () -> driverController.getLeftY() * -1));

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
    operatorController.leftTrigger(0.1).onTrue(new IntakeCommand(intake, outtake, () -> operatorController.getLeftTriggerAxis()));
    operatorController.rightTrigger(0.1).onTrue(new OuttakeCommand(outtake, () -> operatorController.getRightTriggerAxis()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
