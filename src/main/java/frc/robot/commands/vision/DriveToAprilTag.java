package frc.robot.commands.vision;

import frc.robot.Constants.CoralConstants.ReefOffsetAngle;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.commands.drive.DriveToTargetCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class DriveToAprilTag extends LoggingCommand {

    private final DriveSubsystem  driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private int                   targetID = 0;


    public DriveToAprilTag(DriveSubsystem driveSubsystem,
        VisionSubsystem visionSubsystem) {

        this.visionSubsystem = visionSubsystem;
        this.driveSubsystem  = driveSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        targetID = (int) visionSubsystem.getTID();

        // If the ambiguity is not acceptable don't run it
        if (visionSubsystem.getAmbiguity() < VisionConstants.AMBIGUITY_THRESHOLD_MEGATAG)
            return;

        switch (targetID) {
        // Blue Reef
        case 17:
            new DriveToTargetCommand(5.57, 3.233, driveSubsystem);
            new DriveToTargetCommand(6.276, 3.614, driveSubsystem);
            new AlignToReefCommand(driveSubsystem, visionSubsystem, ReefOffsetAngle.LEFT_CORAL);
            break;
        case 18:
            break;
        case 19:
            break;
        case 20:
            break;
        case 21:
            break;
        case 22:
            break;

        // Red Reef
        case 6:
            break;
        case 7:
            break;
        case 8:
            break;
        case 9:
            break;
        case 10:
            break;
        case 11:
            break;

        }



    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

        logCommandEnd(interrupted);

        // Stop the robot if required
        driveSubsystem.setMotorSpeeds(0, 0);
    }
}