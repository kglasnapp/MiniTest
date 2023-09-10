// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.TeleopDriveConstants.DEADBAND;

//import java.lang.ModuleLayer.Controller;
import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultElevatorCommand;
import frc.robot.commands.FieldHeadingDriveCommand;
import frc.robot.commands.DefaultGrabberCommand;
import frc.robot.commands.PositionCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberTiltSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import static frc.robot.utilities.Util.logf;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // TODO Adjust to show correct PID on the Smart Dash Board
  public static ShowPID showPID = ShowPID.TILT;

  public static enum RobotMode {
    Cone, Cube
  }

  public static enum ShowPID {
    NONE, ELEVATOR, TILT
  }

  public static enum OperatorButtons {
    HOME(3), CUBE(2), CONE(1), GROUND(4), CHUTE(5), SHELF(6), LOW(7), MIDDLE(8), HIGH(9);

    public final int value;

    private OperatorButtons(int value) {
      this.value = value;
    }
  }

  private final static CommandXboxController driveController = new CommandXboxController(2);
  private final static CommandXboxController operatorController = new CommandXboxController(3);
  //private final PhotonCamera photonCamera = new PhotonCamera("photonvision");
  private final PhotonCamera photonCamera = null;
  public final GrabberTiltSubsystem grabberSubsystem = new GrabberTiltSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  
  private final Drivetrain drivetrain = new Drivetrain();
  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(photonCamera, drivetrain);

  private final ChaseTagCommand chaseTagCommand = new ChaseTagCommand(photonCamera, drivetrain,
      poseEstimator::getCurrentPose);

  private final FieldHeadingDriveCommand fieldHeadingDriveCommand = new FieldHeadingDriveCommand(
      drivetrain,
      () -> poseEstimator.getCurrentPose().getRotation(),
      () -> -modifyAxis(driveController.getLeftY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(driveController.getLeftX()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -driveController.getRightY(),
      () -> -driveController.getRightX());

  public static boolean smartForElevator = true;
  public static RobotMode robotMode;
  public static LedSubsystem leds = new LedSubsystem();

  //private final  CommandXboxController driveController =controller;
  //private final  CommandXboxController operatorContoller = controller;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    drivetrain.setDefaultCommand(new DefaultDriveCommand(
        drivetrain,
        () -> poseEstimator.getCurrentPose().getRotation(),
        () -> -modifyAxis(driveController.getLeftY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driveController.getLeftX()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(driveController.getRightX()) * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 2));

    setMode(RobotMode.Cone);
    grabberSubsystem.setDefaultCommand(new DefaultGrabberCommand(grabberSubsystem, intakeSubsystem, driveController));
    elevatorSubsystem.setDefaultCommand(new DefaultElevatorCommand(elevatorSubsystem, driveController));
    

    // Configure the button bindings
    configureButtonBindings();
    configureDashboard();
  }

  private void configureDashboard() {

  }

  public static int getDriverPov() {
    return driveController.getHID().getPOV();
  }

  public void setMode(RobotMode mode) {
    robotMode = mode;
    logf("Set Robot Mode: %s\n", mode);
    SmartDashboard.putString("Mode", robotMode.toString());
    leds.setRobotModeLeds();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button resets the robot pose
    driveController.back().onTrue(Commands.runOnce(poseEstimator::resetFieldPosition, drivetrain));
    driveController.y().whileTrue(chaseTagCommand);
    driveController.start().toggleOnTrue(fieldHeadingDriveCommand);
    operatorController.button(OperatorButtons.CONE.value).onTrue(Commands.runOnce(new Runnable() {
      public void run() {
        setMode(RobotMode.Cone);
      }
    }));

    operatorController.button(OperatorButtons.CUBE.value).onTrue(Commands.runOnce(new Runnable() {
      public void run() {
        setMode(RobotMode.Cube);
      }
    }));
    operatorController.button(OperatorButtons.HIGH.value).onTrue(new PositionCommand(this, OperatorButtons.HIGH));
    operatorController.button(OperatorButtons.MIDDLE.value).onTrue(new PositionCommand(this, OperatorButtons.MIDDLE));
    operatorController.button(OperatorButtons.LOW.value).onTrue(new PositionCommand(this, OperatorButtons.LOW));  
    operatorController.button(OperatorButtons.HOME.value).onTrue(new PositionCommand(this, OperatorButtons.HOME));  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        1,
        1)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DrivetrainConstants.KINEMATICS);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these no interior waypoints
        List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, Rotation2d.fromDegrees(90)),
        config);

    return new PrintCommand("Starting auto")
        .andThen(new InstantCommand(
            () -> poseEstimator.setCurrentPose(new Pose2d(0, 0, new Rotation2d(0))), drivetrain))
        .andThen(drivetrain.createCommandForTrajectory(exampleTrajectory, poseEstimator::getCurrentPose))
        .andThen(new RunCommand(drivetrain::stop, drivetrain))
        .andThen(new PrintCommand("Done with auto"));
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
