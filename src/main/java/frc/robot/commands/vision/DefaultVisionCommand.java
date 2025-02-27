package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class DefaultVisionCommand extends LoggingCommand {

    // the target id of the apriltag
    private final int             targetID    = 0;

    private final DriveSubsystem  driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final double          targetX     = 0;
    private final PIDController   controllerX = new PIDController(0.03, 0, 0);

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


        // Add required subsystems
        addRequirements(driveSubsystem);
        controllerX.setTolerance(1);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        logCommandStart();
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double tx = visionSubsystem.getTX();

        if (visionSubsystem.getTV() == 0) {
            System.out.println("NO TARGETS FOUND");
            driveSubsystem.setMotorSpeeds(0, 0);
            return;
        }


        double turn = controllerX.calculate(tx, targetX);

        // Apply constraints to prevent excessive speed
        turn = Math.max(-0.5, Math.min(turn, 0.5)); // Clamp turn speed

        setArcadeDriveMotorSpeeds(0, turn, 1);
        // System.out.println(visionSubsystem.getBotPose());

        // double targetOffsetAngle_Vertical = visionSubsystem.getTY();

        // // how many degrees back is your limelight rotated from perfectly vertical?
        // double limelightMountAngleDegrees = 20;

        // // distance from the center of the Limelight lens to the floor
        // double limelightLensHeightInches = 22.4;

        // // test arpiltag height 30.7cm
        // // from the gorund 21.6cm
        // // angle from horizontal 20 deg

        // // distance from the target to the floor
        // double goalHeightInches = 30.7;

        // double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        // double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        // LL Forward 13.5 cm
        // LL Up 9.5 cm

        // // calculate distance
        // double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
        // / Math.tan(angleToGoalRadians);
        // System.out.println(distanceFromLimelightToGoalInches);
        System.out.println(turn + " " + tx);
        setArcadeDriveMotorSpeeds(0, turn, 1);
    }

    private void setArcadeDriveMotorSpeeds(double speed, double turn, double driveScalingFactor) {

        // Cut the spin in half because it will be applied to both sides.
        // Spinning at 1.0, should apply 0.5 to each side.
        turn = turn / 2.0;

        // Keep the turn, and reduce the forward speed where required to have the
        // maximum turn.
        if (Math.abs(speed) + Math.abs(turn) > 1.0) {
            speed = (1.0 - Math.abs(turn)) * Math.signum(speed);
        }

        double leftSpeed  = (speed + turn) * driveScalingFactor;
        double rightSpeed = (speed - turn) * driveScalingFactor;

        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        if (controllerX.atSetpoint())
            return true;
        if (hasElapsed(2.5f)) {
            setFinishReason("Timed out");
            return true;
        }

        return false;

        // Ivan was here

    }

    @Override
    public void end(boolean interrupted) {

        logCommandEnd(interrupted);

        // Stop the robot if required
        if (true) {
            driveSubsystem.setMotorSpeeds(0, 0);
        }
    }
}