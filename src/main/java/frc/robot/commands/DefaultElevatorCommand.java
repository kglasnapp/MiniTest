package frc.robot.commands;

import static frc.robot.Util.logf;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.subsystems.ElevatorSubsystem;

public class DefaultElevatorCommand extends CommandBase {
    static ElevatorSubsystem elevatorSubsystem;
    CommandXboxController operatorController;
    int lastPov = -1;

    enum STATE {
        IDLE, HOMEING, RAISED, DROPED
    }

    STATE state = STATE.IDLE;

    public DefaultElevatorCommand(ElevatorSubsystem elevatorSubsystem, CommandXboxController operatorController) {
        DefaultElevatorCommand.elevatorSubsystem = elevatorSubsystem;
        this.operatorController = operatorController;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        logf("Init Grabber Default %d\n", Robot.count);
    }

    @Override
    public void execute() {
        boolean left = operatorController.getHID().getRawButtonPressed(5);
        boolean right = operatorController.getHID().getRawButtonPressed(6);
        double pos = elevatorSubsystem.getLastElevatorPositionInches();

        if (left) {
            elevatorSubsystem.setElevatorPos(pos + 3);
        }

        if (right) {
            elevatorSubsystem.setElevatorPos(pos - 3);
        }
    }
}
