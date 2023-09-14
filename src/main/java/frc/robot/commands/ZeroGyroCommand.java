package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Util.logf;

public class ZeroGyroCommand extends CommandBase {
    Drivetrain m_drivetrainSubsystem;
    BalanceCommand balanceCommand;
    double currentOrientation;

    public ZeroGyroCommand(Drivetrain drivetrainSubsystem, BalanceCommand balanceCommand, double currentOrientation) {
        logf("Init -- Zero Gyro Command\n");
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.currentOrientation = currentOrientation;
        this.balanceCommand = balanceCommand;
    }

    @Override
    public void execute() {
        m_drivetrainSubsystem.zeroGyroscope(currentOrientation);
        balanceCommand.zeroGyroscope();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
