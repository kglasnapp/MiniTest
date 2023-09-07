// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import static frc.robot.Constants.AutoConstants.THETA_CONSTRAINTS;

/** Represents a differential drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  private final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS;
  private static final double kWheelRadius = DrivetrainConstants.DRIVETRAIN_WHEEL_DIAMETER_METERS / 2; // meters
  private static final int kEncoderResolution = 4096;

  private final MotorController leftLeader = new WPI_TalonSRX(2);
  //private final MotorController leftFollower = new WPI_TalonSRX(3);
  private final MotorController rightLeader = new WPI_TalonSRX(3);
  //private final MotorController rightFollower = new PWMSparkMax(4);

  private final Encoder leftEncoder = new Encoder(0, 1);
  private final Encoder rightEncoder = new Encoder(2, 3);

  //private final MotorControllerGroup leftGroup = new MotorControllerGroup(leftLeader, leftFollower);
  //private final MotorControllerGroup rightGroup = new MotorControllerGroup(rightLeader, rightFollower);

  private final MotorControllerGroup leftGroup = new MotorControllerGroup(leftLeader);
  private final MotorControllerGroup rightGroup = new MotorControllerGroup(rightLeader);

  private final PIDController leftPIDController = new PIDController(1, 0, 0);
  private final PIDController rightPIDController = new PIDController(1, 0, 0);

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth);

  /*
   * Here we use DifferentialDrivePoseEstimator so that we can fuse odometry
   * readings. The numbers used below are robot specific, and should be tuned.
   */
  private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(kinematics,
      navx.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance(), new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)), VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1, 3);

  private DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds();

  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse
   * and resets the gyro.
   */
  public Drivetrain() {
    navx.reset();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightGroup.setInverted(true);

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    leftEncoder.reset();
    rightEncoder.reset();
  }

  /**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput = leftPIDController.calculate(leftEncoder.getRate(), speeds.leftMetersPerSecond);
    final double rightOutput = rightPIDController.calculate(rightEncoder.getRate(), speeds.rightMetersPerSecond);
    leftGroup.setVoltage(leftOutput + leftFeedforward);
    rightGroup.setVoltage(rightOutput + rightFeedforward);
    SmartDashboard.putNumber("Left Volts", leftOutput + leftFeedforward);
    SmartDashboard.putNumber("Right Volts", rightOutput + rightFeedforward);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot    Angular velocity in rad/s.
   */
  public void drive(double xSpeed, double rot) {
    wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  public void drive(ChassisSpeeds x) {
    wheelSpeeds = kinematics.toWheelSpeeds(x);
    setSpeeds(wheelSpeeds);
  }

  public ChassisSpeeds getChassisSpeeds() {
    ChassisSpeeds  x = new ChassisSpeeds();
    ChassisSpeeds.fromFieldRelativeSpeeds(wheelSpeeds.rightMetersPerSecond, wheelSpeeds.leftMetersPerSecond, 9.0,
        navx.getRotation2d());
    return x;
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    poseEstimator.update(navx.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());

    // Also apply vision measurements. We use 0.3 seconds in the past as an example
    // -- on
    // a real robot, this must be calculated based either on latency or timestamps.
    poseEstimator.addVisionMeasurement(
        ExampleGlobalMeasurementSensor.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition()),
        Timer.getFPGATimestamp() - 0.3);
  }

  public double getRight() {
    return rightEncoder.getDistance();
  }

  public double getLeft() {
    return leftEncoder.getDistance();
  }

  public Rotation2d getGyroscopeRotation() {
    return navx.getRotation2d();
  }

  public void stop() {
    drive(0, navx.getAngle());
  }

  public Command createCommandForTrajectory(Trajectory trajectory, Supplier<Pose2d> poseSupplier) {
    var thetaController = new ProfiledPIDController(AutoConstants.THETA_kP, AutoConstants.THETA_kI,
        AutoConstants.THETA_kD, THETA_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand =
    // new SwerveControllerCommand(
    // trajectory,
    // poseSupplier,
    // DrivetrainConstants.KINEMATICS,
    // new PIDController(AutoConstants.X_kP, AutoConstants.X_kI,
    // AutoConstants.X_kD),
    // new PIDController(AutoConstants.Y_kP, AutoConstants.Y_kI,
    // AutoConstants.Y_kD),
    // thetaController,
    // this::setModuleStates,
    // this);
    Command x = new InstantCommand();

    return x;
  }
}
