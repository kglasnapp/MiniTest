package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.Util.logf;

public class ZeroGyroCommand extends CommandBase {
    DrivetrainSubsystem m_drivetrainSubsystem;
    BalanceCommand balanceCommand;
    double currentOrientation;

    public ZeroGyroCommand(DrivetrainSubsystem drivetrainSubsystem, BalanceCommand balanceCommand, double currentOrientation) {
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
