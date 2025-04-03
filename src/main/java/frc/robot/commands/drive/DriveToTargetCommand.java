package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class DriveToTargetCommand extends SequentialCommandGroup {

    public DriveToTargetCommand(double targetX, double targetY, DriveSubsystem driveSubsystem) {

        addCommands(new RotateToTargetCommand(targetX, targetY, driveSubsystem));
        addCommands(new ForwardToTargetCommand(targetX, targetY, driveSubsystem));
    }

}