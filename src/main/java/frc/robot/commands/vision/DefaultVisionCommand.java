package frc.robot.commands.vision;

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

        addRequirements(visionSubsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // The lesser the ambiguity the better acuracy
        // if (visionSubsystem.getAmbiguity() > VisionConstants.AMBIGUITY_THRESHOLD_MEGATAG) {
        // System.out.println("Using Limelight: " + visionSubsystem.getAmbiguity());
        // driveSubsystem.setPose(visionSubsystem.getBotPose());
        // return;
        // }

        // System.out.println("Using Encoders");
        driveSubsystem.updateOdometry();
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