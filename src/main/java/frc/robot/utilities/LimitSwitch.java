package frc.robot.utilities;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LedSubsystem.Leds;

public class LimitSwitch {
    private SparkMaxLimitSwitch forwardLimit;
    private SparkMaxLimitSwitch reverseLimit;
    boolean lastForward = true;
    boolean lastReverse = true;
    Leds forwardLed;
    Leds reverseLed;
    String forwardName;
    String reverseName;

    public LimitSwitch(CANSparkMax motor, String name, Leds forwardLed, Leds reverseLed) {
        this.forwardLed = forwardLed;
        this.reverseLed = reverseLed;
        forwardLimit = motor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        reverseLimit = motor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        forwardLimit.enableLimitSwitch(true);
        reverseLimit.enableLimitSwitch(true);
        forwardName = name + "for";
        reverseName = name + "rev";
    }

    public void periodic() {
        boolean forLimit = forwardLimit.isPressed();
        boolean revLimit = reverseLimit.isPressed();
        if (forLimit != lastForward) {
            RobotContainer.leds.setLimitSwitchLed(forwardLed, forLimit);
            SmartDashboard.putBoolean(forwardName, forLimit);
            lastForward = forLimit;
        }
        if (revLimit != lastReverse) {
            RobotContainer.leds.setLimitSwitchLed(reverseLed, revLimit);
            SmartDashboard.putBoolean(reverseName, revLimit);
            lastReverse = revLimit;
        }
    }

    public boolean getForward() {
        return lastForward;
    }

    public boolean getReverse() {
        return lastReverse;
    }
}