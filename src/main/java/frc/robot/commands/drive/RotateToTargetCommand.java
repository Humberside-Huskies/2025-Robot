package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;

public class RotateToTargetCommand extends LoggingCommand {

    private final double         targetX;
    private final double         targetY;

    private final PIDController  headingController = new PIDController(0.01, 0, 0);

    private double               angleToTarget;

    private final DriveSubsystem driveSubsystem;


    public RotateToTargetCommand(double targetX, double targetY, DriveSubsystem driveSubsystem) {

        this.targetX        = targetX;
        this.targetY        = targetY;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);

        headingController.setSetpoint(0);
        headingController.setTolerance(0.8);
    }


    @Override
    public void initialize() {
        logCommandStart();

        Pose2d currPose = driveSubsystem.getPose();
        System.out.println("Vladislav " + (targetY - currPose.getY()) + ", " + (targetX - currPose.getX()));
        angleToTarget = 360 - Math.toDegrees(Math.atan2(targetY - currPose.getY(), targetX - currPose.getX()));
    }

    @Override
    public void execute() {
        double headingError = driveSubsystem.getHeadingError(angleToTarget);
        double heading      = headingController.calculate(headingError);
        heading = Math.min(Math.max(heading, -0.1), 0.1);

        driveSubsystem.setMotorSpeeds(-heading, heading);
    }

    @Override
    public boolean isFinished() {
        // when the robots heading error is close to 0 with tolerance
        return headingController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {

        logCommandEnd(interrupted);
    }
}