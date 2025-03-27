package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class AlignToAprilTagCommand extends LoggingCommand {

    // the target id of the apriltag
    private double              targetID    = 0.0;

    private DriveSubsystem      driveSubsystem;
    private VisionSubsystem     visionSubsystem;

    private final double        targetX     = 0;
    private double              targetD     = .5;
    private final PIDController controllerX = new PIDController(0.01, 0, 0);
    private final PIDController controllerD = new PIDController(.01, 0, 0);


    /**
     * DriveForTime command drives at the specified heading at the specified speed for the specified
     * time.
     *
     * @param speed in the range -1.0 to +1.0
     * @param driveSubsystem
     */

    public AlignToAprilTagCommand(DriveSubsystem driveSubsystem,
        VisionSubsystem visionSubsystem) {

        this.visionSubsystem = visionSubsystem;
        this.driveSubsystem  = driveSubsystem;

        // Tony is gay
        // Tony is gay

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
        targetID = visionSubsystem.getTID();
        double ty = visionSubsystem.getTY();

        if (visionSubsystem.getTV() == 0) {
            System.out.println("NO TARGETS FOUND");
            driveSubsystem.setMotorSpeeds(0, 0);
            return;
        }



        double turn    = controllerX.calculate(-tx, targetX);

        double forward = controllerD.calculate(distanceCalculator(ty, targetID), targetD);

        // Apply constraints to prevent excessive speed turn = Math.max(-0.5, Math.min(turn, 0.5)); // Clamp turn speed

        setArcadeDriveMotorSpeeds(forward, turn, .7);

    }

    private double findGoalHeightMeter(double targetID) {
        int    ID               = (int) targetID;
        double goalHeightMeters = 0;

        switch (ID) {
        case 22:
            goalHeightMeters = VisionConstants.ReefHeightMeters;
            break;
        case 21:
            goalHeightMeters = VisionConstants.ReefHeightMeters;
            break;
        case 19:
            goalHeightMeters = VisionConstants.ReefHeightMeters;
            break;
        case 18:
            goalHeightMeters = VisionConstants.ReefHeightMeters;
            break;
        case 17:
            goalHeightMeters = VisionConstants.ReefHeightMeters;
            break;
        case 16:
            goalHeightMeters = VisionConstants.ProcessorHeightMeters;
            break;
        case 15:
            goalHeightMeters = VisionConstants.BargeHeightMeters;
            break;
        case 14:
            goalHeightMeters = VisionConstants.BargeHeightMeters;
            break;
        case 13:
            goalHeightMeters = VisionConstants.StationHeightMeters;
            break;
        case 12:
            goalHeightMeters = VisionConstants.StationHeightMeters;
            break;
        case 11:
            goalHeightMeters = VisionConstants.ReefHeightMeters;
            break;
        case 10:
            goalHeightMeters = VisionConstants.ReefHeightMeters;
            break;
        case 9:
            goalHeightMeters = VisionConstants.ReefHeightMeters;
            break;
        case 8:
            goalHeightMeters = VisionConstants.ReefHeightMeters;
            break;
        case 7:
            goalHeightMeters = VisionConstants.ReefHeightMeters;
            break;
        case 6:
            goalHeightMeters = VisionConstants.ReefHeightMeters;
            break;
        case 5:
            goalHeightMeters = VisionConstants.BargeHeightMeters;
            break;
        case 4:
            goalHeightMeters = VisionConstants.BargeHeightMeters;
            break;
        case 3:
            goalHeightMeters = VisionConstants.ProcessorHeightMeters;
            break;
        case 2:
            goalHeightMeters = VisionConstants.StationHeightMeters;
            break;
        case 1:
            goalHeightMeters = VisionConstants.StationHeightMeters;
            break;

        default:
            break;
        }

        return goalHeightMeters;
    }

    private double distanceCalculator(double ty, double ID) {

        double targetOffsetAngle_Vertical = ty;

        // distance from the target to the floor
        double goalHeightMeters           = findGoalHeightMeter(ID);


        double angleToGoalDegrees         = VisionConstants.mountedAngleDegrees + targetOffsetAngle_Vertical;

        // calculate distance
        double distanceFromGoal           = (goalHeightMeters - VisionConstants.mountedHeightMeters)
            / Math.tan(angleToGoalDegrees);

        return distanceFromGoal;
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