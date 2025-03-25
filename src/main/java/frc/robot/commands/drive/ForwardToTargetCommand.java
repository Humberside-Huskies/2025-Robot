package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;

public class ForwardToTargetCommand extends LoggingCommand {

    private final double         targetX;
    private final double         targetY;

    private final PIDController  xController = new PIDController(0.1, 0, 0);
    private final PIDController  yController = new PIDController(0.1, 0, 0);

    private final DriveSubsystem driveSubsystem;

    public ForwardToTargetCommand(double targetX, double targetY, DriveSubsystem driveSubsystem) {
        this.targetX        = targetX;
        this.targetY        = targetY;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);

        // set tolerances of x and y
        xController.setTolerance(1);
        yController.setTolerance(1);
    }

    @Override
    public void execute() {
        Pose2d currPose = driveSubsystem.getPose();

        // calculate the change in pos (velocity)
        double xvel     = xController.calculate(currPose.getX(), targetX);
        double yvel     = yController.calculate(currPose.getY(), targetY);

        // calculate the magnitude of velocity
        double speed    = Math.sqrt(xvel * xvel + yvel * yvel);
        System.out.println(currPose.getX() + " " + currPose.getY());

        setArcadeDriveMotorSpeeds(speed, 0, DriveConstants.DRIVE_SCALING_NORMAL);
    }

    /**
     * Calculate the scaled arcade drive speeds from the passed in values. In arcade mode, the turn
     * is cut in half to help control the robot more consistently.
     *
     * @param speed
     * @param turn
     * @param driveScalingFactor
     */
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

    @Override
    public boolean isFinished() {
        // when the robot is close enough to target with tolerance
        return xController.atSetpoint() && yController.atSetpoint();
    }
}