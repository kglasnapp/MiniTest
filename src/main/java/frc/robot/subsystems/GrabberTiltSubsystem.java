package frc.robot.subsystems;

import static frc.robot.Util.logf;
import static frc.robot.Util.round2;
import static frc.robot.Util.normalizeAngle;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.ShowPID;
import frc.robot.subsystems.LedSubsystem.Leds;
import frc.robot.utilities.LimitSwitch;

/**
 * REV Smart Motion Guide
 * 
 * The SPARK MAX includes a control mode, REV Smart Motion which is used to 
 * control the position of the motor, and includes a max velocity and max 
 * acceleration parameter to ensure the motor moves in a smooth and predictable 
 * way. This is done by generating a motion profile on the fly in SPARK MAX and 
 * controlling the velocity of the motor to follow this profile.
 * 
 * Since REV Smart Motion uses the velocity to track a profile, there are only 
 * two steps required to configure this mode:
 *    1) Tune a velocity PID loop for the mechanism
 *    2) Configure the smart motion parameters
 * 
 * Tuning the Velocity PID Loop
 * 
 * The most important part of tuning any closed loop control such as the velocity 
 * PID, is to graph the inputs and outputs to understand exactly what is happening. 
 * For tuning the Velocity PID loop, at a minimum we recommend graphing:
 *
 *    1) The velocity of the mechanism (‘Process variable’)
 *    2) The commanded velocity value (‘Setpoint’)
 *    3) The applied output
 *
 * This example will use ShuffleBoard to graph the above parameters. Make sure to
 * load the shuffleboard.json file in the root of this directory to get the full
 * effect of the GUI layout.
 */

public class GrabberTiltSubsystem extends SubsystemBase {
    private static final int GRABBER_TILT_MOTOR_ID = 12;
    // this is the conversion ratio from the absolute encoder to the relative encoder of 
    // the tilt motor 
    final static double ABSOLUTE_ENCODER_RATIO = 360;

    private double lastTiltRotations;
    private double lastTiltAngle = 0;
    private CANSparkMax grabberTiltMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder tiltEncoder;
    private PID_MAX pid = new PID_MAX();
    private CANCoder absEncoder;
    private double rotationsPerDegree = 1;  // TODO fix when connected to Sibling
    private LimitSwitch limitSwitch;

    public GrabberTiltSubsystem() {

        // Setup paramters for the tilt motor
        grabberTiltMotor = new CANSparkMax(GRABBER_TILT_MOTOR_ID, MotorType.kBrushless);
        grabberTiltMotor.restoreFactoryDefaults();
        grabberTiltMotor.setSmartCurrentLimit(2);

        limitSwitch = new LimitSwitch(grabberTiltMotor, "Tlt", Leds.GrabberForward, Leds.GrabberReverse);
        absEncoder = new CANCoder(9);
        tiltEncoder = grabberTiltMotor.getEncoder();
        tiltEncoder.setPosition(absEncoder.getAbsolutePosition() / ABSOLUTE_ENCODER_RATIO);
        pidController = grabberTiltMotor.getPIDController();

        pid.PIDCoefficientsTilt(pidController);
        pid.PIDToMax();
        pid.putPidCoefficientToDashBoard();
        /* Angle Encoder Config */

        //setTiltAngle(0);
        logf("Grabber System Setup kP for Tilt:%.6f Conversion Factor:%.2f Counts per Rev:%d\n", pid.kP,
                tiltEncoder.getPositionConversionFactor(), tiltEncoder.getCountsPerRevolution());

        // Limit currnet for Testing
        grabberTiltMotor.setSmartCurrentLimit(2);
    }

    public boolean setTiltAngle(double angle) {
        if (angle < 0 || angle > 50) {
            logf("****** Error attempted to set an angle to large or small angle:%.1f\n", angle);
            return false;
        }

        double setPointRotations = angle * rotationsPerDegree;
        lastTiltAngle = angle;
        lastTiltRotations = setPointRotations;

        logf("Set tilt angle:%.2f set point:%f kp:%f\n", angle, setPointRotations, pidController.getP());
        pidController.setReference(setPointRotations, CANSparkMax.ControlType.kSmartMotion);
        SmartDashboard.putNumber("Tilt SPR", setPointRotations);
        SmartDashboard.putNumber("Tilt Ang", angle);
        return true;
    }

    public double getLastTiltAngle() {
        return lastTiltAngle;
    }

    public void setBrakeMode(boolean mode) {
        grabberTiltMotor.setIdleMode(mode ? IdleMode.kBrake : IdleMode.kCoast);
        logf("Brake mode: %s\n", grabberTiltMotor.getIdleMode());
    }

    public double getTiltRotations() {
        return tiltEncoder.getPosition();
    }

    public double getTiltAngleDegrees() {
        return tiltEncoder.getPosition() / rotationsPerDegree;
    }

    public double getLastTiltPos() {
        return lastTiltRotations;
    }

    public double getTiltCurrent() {
        return grabberTiltMotor.getOutputCurrent();
    }

    public boolean getForwardLimitSwitchTilt() {
        return limitSwitch.getForward();
    }

    public boolean getReverseLimitSwitchTilt() {
        return limitSwitch.getReverse();
    }

    public boolean atSetPoint() {
        double error = tiltEncoder.getPosition() - lastTiltRotations;
        if (Robot.count % 10 == 5) {
            SmartDashboard.putNumber("TiltErr", error);
        }
        // Note error is in revolutions
        return Math.abs(error) < .1;
    }

    // If grabber retracted it is safe to move the elevator
    public boolean retracted() {
        return Math.abs(normalizeAngle(absEncoder.getAbsolutePosition())) < 5;
    }

    // If grabber retracted it is safe to move the elevator
    public double getAbsEncoder() {
        return absEncoder.getAbsolutePosition();
    }

    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        limitSwitch.periodic();
        if (Robot.count % 250 == 0) {
            // we adjust the relative encoder of the tilt
            // motor every 5 secs
            double tiltAbsolutePosition = getAbsEncoder();
            // TODO put back in when connected to actual intake
            // tiltEncoder.setPosition(tiltAbsolutePosition / ABSOLUTE_ENCODER_RATIO);
            logf("Motor Tilt Angle:%.3f Abs Encoder Rotations:%.3f Angle:%.3f\n", getTiltAngleDegrees(), tiltAbsolutePosition / ABSOLUTE_ENCODER_RATIO,
                     getAbsEncoder());
        }
        if (RobotContainer.smartForElevator) {
            if (Robot.count % 15 == 5) {
                double current = getTiltCurrent();
                SmartDashboard.putNumber("TltCur", round2(current));
                SmartDashboard.putNumber("TltAng", round2(getTiltAngleDegrees()));
                SmartDashboard.putNumber("TltLastRot", lastTiltRotations);
                SmartDashboard.putNumber("TltPwr", round2(grabberTiltMotor.getAppliedOutput()));
            }
            if (Robot.count % 30 == 10) {
                if (RobotContainer.showPID == ShowPID.TILT) {
                    pid.getPidCoefficientsFromDashBoard();
                }
                SmartDashboard.putNumber("AbsTltAng", round2(absEncoder.getAbsolutePosition()));
            }
        }
    }
}
