package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static frc.robot.utilities.Util.logf;

public class DriveToObjectCommand extends CommandBase {
    private String type; // Object to look for
    private DrivetrainSubsystem drivetrainSubsystem;
    private CoralSubsystem coral;
    private double x;
    private double area;
    private double finishArea = 50;
    private double finishX = 0.02;

    /** Creates a new ReplaceMeCommand. */
    public DriveToObjectCommand(DrivetrainSubsystem drivetrainSubsystem, String type) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
        this.type = type;
        coral = RobotContainer.coralSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        coral.setLookForType(type);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double omegaSpeed = 0;
        double xSpeed = 0;
        x = coral.x;
        if (x > .02) {
            omegaSpeed = x / 64;
        }
        area = coral.area;
        if (area < 50) {
            xSpeed = .1;
        }
        if (Robot.count % 10 == 5) {
            logf("Coral x:%.2f y:%.2f area:%.0f\n", x, coral.y, area);
        }
        drivetrainSubsystem.drive(new ChassisSpeeds(xSpeed, 0, Math.toRadians(omegaSpeed)));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (x < finishX && coral.area > finishArea);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0, 0, 0));
        coral.setLookForType(type);
    }
}