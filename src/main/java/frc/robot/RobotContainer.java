// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
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
  private final OuttakeSubsystem outtake = new OuttakeSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final DrivebaseSubsystem drivebase = new DrivebaseSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.driverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.operatorControllerPort);

  // create here cuz "Loop time of 0.02s overrun" if in autonomous init
  // apparently creating the trajectory and LTVUnicycleController takes a while
  private final Command trajectoryCommand = createTestTrajectoryCommand();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // left trigger axis is definitely not the climber axis i just need a placeholder
    climber.setDefaultCommand(new ClimberCommand(climber, operatorController::getLeftTriggerAxis));
    intake.setDefaultCommand(new IntakePivotCommand(intake, () -> operatorController.getLeftY() * -1));
    outtake.setDefaultCommand(new OuttakePivotCommand(outtake, () -> operatorController.getRightY() * -1));
    drivebase.setDefaultCommand(new FieldCentricDriveCommand(drivebase, () -> {
      // different controls for forward and backward for some reason, if backward axis is moved forward then robot moves backward or something
      // backward overrides forward perhaps
      double forwardSpeed = driverController.getRightTriggerAxis();
      double backwardSpeed = driverController.getLeftTriggerAxis();
      return backwardSpeed > 0 ? -backwardSpeed : forwardSpeed;
    }, driverController::getLeftX, driverController::getLeftY, () -> driverController.rightBumper().getAsBoolean()));

    drawRobot();

    // Configure the trigger bindings
    configureBindings();
  }

  private void drawRobot() {
    Mechanism2d mechanism = new Mechanism2d(3, 4);
    MechanismRoot2d mechRoot = mechanism.getRoot("root", 0.5, 0.1);

    // chassis ligament
    mechRoot.append(new MechanismLigament2d("chassis", 2, 0));
  
    // tower ligaments
    MechanismRoot2d towerRoot = mechanism.getRoot("tower root", 2, 0.2);
    MechanismLigament2d verticalTower = towerRoot.append(new MechanismLigament2d("vertical tower", 2, 90, 6, new Color8Bit(0, 0, 255)));
    MechanismLigament2d diagonalTower = verticalTower.append(new MechanismLigament2d("diagonal tower", 2, 148, 6, new Color8Bit(0, 0, 255)));
    diagonalTower.append(new MechanismLigament2d("short vertical tower", 0.3, 32, 6, new Color8Bit(0, 0, 255)));
    this.outtake.setBaseLigament(verticalTower);
  
    MechanismRoot2d intakeRoot = mechanism.getRoot("intake root", 2.4, 0.2);
    MechanismLigament2d intakeHolderLigament = intakeRoot.append(new MechanismLigament2d("intake holder", 0.5, 90, 6, new Color8Bit(255, 0, 0)));
    this.intake.setBaseLigament(intakeHolderLigament);
  
    SmartDashboard.putData("mechanism", mechanism);
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
    operatorController.leftTrigger(0.1).whileTrue(new IntakeCommand(intake, operatorController::getLeftTriggerAxis));

    // OuttakeCommand originally was operatorController.rightTrigger(0.1).onTrue
    // changed to whileTrue because once the right trigger exceeds 0.1, OuttakeCommand will run for eternity
    // this means the outtake subsystem is occupied and cannot be used by OuttakePivotCommand
    // not sure if this is intentional but me want pivot work
    operatorController.rightTrigger(0.1).whileTrue(new OuttakeCommand(outtake, operatorController::getRightTriggerAxis));
    operatorController.rightBumper().whileTrue(new EjectCommand(intake));
    operatorController.b().onTrue(new MagicRotCommand(intake, "intake", 0, 0.25));

    if (RobotBase.isSimulation()) {
      operatorController.a().onTrue(new InstantCommand(drivebase::resetSimPos));
    }
  }

  private Command createTestTrajectoryCommand() {
    DifferentialDriveVoltageConstraint constraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        DriveConstants.ksVolts,
        DriveConstants.kvVoltsSecsPerMeter,
        DriveConstants.kaVoltSecsSquaredPerMeter
      ),
      DriveConstants.kDriveKinematics,
      10
    );

    TrajectoryConfig config = new TrajectoryConfig(
      DriveConstants.kMaxSpeedMetersPerSecond,
      DriveConstants.kMaxAccelerationMetersPerSecondSquared
    ).setKinematics(DriveConstants.kDriveKinematics).addConstraint(constraint);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      DriveConstants.simDefaultPose,
      List.of(new Translation2d(DriveConstants.simDefaultPose.getX() + 1, DriveConstants.simDefaultPose.getY() + 1), new Translation2d(DriveConstants.simDefaultPose.getX() + 2, DriveConstants.simDefaultPose.getY() - 1)),
      new Pose2d(DriveConstants.simDefaultPose.getX() + 3, DriveConstants.simDefaultPose.getY(), new Rotation2d()),
      config
    );

    drivebase.field.getObject("Trajectory").setTrajectory(trajectory);
    return new FollowTrajectoryCommand(drivebase, trajectory);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return this.trajectoryCommand;
  }
}
