package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberTiltSubsystem;

public class GrabberCommand extends CommandBase {
    GrabberTiltSubsystem grabber;

    public GrabberCommand(GrabberTiltSubsystem grabber, boolean action) {
        this.grabber = grabber;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
