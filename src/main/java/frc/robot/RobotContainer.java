// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final ClimberSubsystem climber = new ClimberSubsystem();
  // private final OuttakeSubsystem outtake = new OuttakeSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final DrivebaseSubsystem drivebase = new DrivebaseSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.driverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.operatorControllerPort);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // left trigger axis is definitely not the climber axis i just need a placeholder
    climber.setDefaultCommand(new ClimberCommand(climber, operatorController::getLeftTriggerAxis));
    intake.setDefaultCommand(new IntakePivotCommand(intake, () -> operatorController.getLeftY() * -1));
    // outtake.setDefaultCommand(new OuttakePivotCommand(outtake, () -> operatorController.getRightY() * -1));
    drivebase.setDefaultCommand(new FieldCentricDriveCommand(drivebase, () -> {
      // different controls for forward and backward for some reason, if backward axis is moved forward then robot moves backward or something
      // backward overrides forward perhaps
      double forwardSpeed = driverController.getRightTriggerAxis();
      double backwardSpeed = driverController.getLeftTriggerAxis();
      return backwardSpeed > 0 ? -backwardSpeed : forwardSpeed;
    }, driverController::getLeftX, driverController::getLeftY, () -> driverController.rightBumper().getAsBoolean()));

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
    operatorController.leftTrigger(0.1).whileTrue((new IntakeCommand(intake, operatorController::getLeftTriggerAxis)));
    // operatorController.rightTrigger(0.1).onTrue(new OuttakeCommand(outtake, () -> operatorController.getRightTriggerAxis()));
    operatorController.rightBumper().whileTrue(new EjectCommand(intake));
    operatorController.b().onTrue(new ClimberMagicButtonCommand(climber));

    if (RobotBase.isSimulation()) {
      operatorController.a().onTrue(new InstantCommand(drivebase::resetSimPos));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new FieldCentricDriveCommand(drivebase, () -> -0.3, () -> 0.0, () -> 0.0, () -> false).withTimeout(2
    );
  }
}
