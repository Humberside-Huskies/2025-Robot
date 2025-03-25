package frc.robot.commands.vision;

import frc.robot.Constants.VisionConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class DefaultVisionCommand extends LoggingCommand {

    private final DriveSubsystem  driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    /**
     * DriveForTime command drives at the specified heading at the specified speed for the specified
     * time.
     *
     * @param speed in the range -1.0 to +1.0
     * @param driveSubsystem
     */

    public DefaultVisionCommand(DriveSubsystem driveSubsystem,
        VisionSubsystem visionSubsystem) {

        this.visionSubsystem = visionSubsystem;
        this.driveSubsystem  = driveSubsystem;

<<<<<<< Updated upstream
        // Add required subsystems
        addRequirements(visionSubsystem);
=======
        addRequirements(visionSubsystem);

>>>>>>> Stashed changes
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
<<<<<<< Updated upstream
        // if (visionSubsystem.getAmbiguity() > VisionConstants.AMBIGUITY_THRESHOLD_MEGATAG) {
        // driveSubsystem.setPose(visionSubsystem.getBotPose());
        // return;
        // }

        // System.out.println("Using Encoders");
        // driveSubsystem.updateOdometry();
=======

        if (visionSubsystem.getAmbiguity() > VisionConstants.AMBIGUITY_THRESHOLD_MEGATAG) {
            System.out.println("Using Limelight");
            driveSubsystem.setPose(visionSubsystem.getBotPose());
            return;
        }

        System.out.println("Using Encoders");
        driveSubsystem.updateOdometry();
>>>>>>> Stashed changes
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

        logCommandEnd(interrupted);
    }
}