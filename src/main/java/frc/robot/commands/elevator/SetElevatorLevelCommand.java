package frc.robot.commands.elevator;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorLevelCommand extends LoggingCommand {

    private ElevatorSubsystem elevatorSubsystem;

    private double            targetEncoderCounts;
    private double            error;

    /**
     * Creates a new ExampleCommand.
     *
     * @param climbSubsystem The subsystem used by this command.
     */
    public SetElevatorLevelCommand(double targetEncoderCounts, ElevatorSubsystem elevatorSubsystem) {
        this.targetEncoderCounts = targetEncoderCounts;
        this.elevatorSubsystem   = elevatorSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        logCommandStart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        error = targetEncoderCounts - elevatorSubsystem.getEncoder();

        if (Math.abs(error) < ElevatorConstants.SLOW_TOLERANCE) {
            if (error > 0)
                elevatorSubsystem.setMotorSpeeds(ElevatorConstants.SLOW_SPEED + ElevatorConstants.HOLD_SPEED);
            else
                elevatorSubsystem.setMotorSpeeds(-ElevatorConstants.SLOW_SPEED);
        }
        else {
            if (error > 0)
                elevatorSubsystem.setMotorSpeeds(ElevatorConstants.FAST_SPEED + ElevatorConstants.HOLD_SPEED);
            else
                elevatorSubsystem.setMotorSpeeds(-ElevatorConstants.FAST_SPEED);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.abs(error) < ElevatorConstants.HOLD_TOLERANCE)
            return true;
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }
}