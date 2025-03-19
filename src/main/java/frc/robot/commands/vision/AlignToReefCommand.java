package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.VisionConstants.ReefOffsetAngle;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;;



public class AlignToReefCommand extends LoggingCommand {


    private DriveSubsystem      driveSubsystem;
    private VisionSubsystem     visionSubsystem;
    private final double        targetX;
    private final PIDController controllerX = new PIDController(0.01, 0, 0);


    /**
     * DriveForTime command drives at the specified heading at the specified speed for the specified
     * time.
     *
     * @param speed in the range -1.0 to +1.0
     * @param driveSubsystem
     */

    public AlignToReefCommand(DriveSubsystem driveSubsystem,
        VisionSubsystem visionSubsystem, ReefOffsetAngle direction) {

        this.visionSubsystem = visionSubsystem;
        this.driveSubsystem  = driveSubsystem;
        this.targetX         = direction.getOffset();
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

        if (tx == 0) {
            // if (visionSubsystem.getTV() == 0) {
            System.out.println("NO TARGETS FOUND");
            driveSubsystem.setMotorSpeeds(0, 0);
            return;
        }

        double turn = controllerX.calculate(tx, targetX);
        System.out.println(turn);
        // Apply constraints to prevent excessive speed
        turn = Math.max(-0.5, Math.min(turn, 0.5)); // Clamp turn speed
        setArcadeDriveMotorSpeeds(0, turn, .7);
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

        if (controllerX.atSetpoint()) {
            setFinishReason("Target Within Tolerance");
            return true;
        }
        // if (hasElapsed(2.0f)) {
        // setFinishReason("Timed out");
        // return true;
        // }

        return false;
    }

    @Override
    public void end(boolean interrupted) {

        logCommandEnd(interrupted);

        // Stop the robot
        driveSubsystem.setMotorSpeeds(0, 0);
    }
}