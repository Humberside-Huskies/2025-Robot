package frc.robot.commands.coral;

import frc.robot.Constants.CoralConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;

public class CoralCommand extends LoggingCommand {

    private CoralSubsystem coralSubsystem;
    private boolean        isIntake;

    /**
     * Creates a new ExampleCommand.
     *
     * @param climbSubsystem The subsystem used by this command.
     */
    public CoralCommand(CoralSubsystem coralSubsystem, boolean isIntake) {

        this.coralSubsystem = coralSubsystem;
        this.isIntake       = isIntake;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(coralSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (isIntake) {
            coralSubsystem.setMotorSpeeds(CoralConstants.INTAKE_SPEED);
        }
        else {
            coralSubsystem.setMotorSpeeds(-CoralConstants.INTAKE_SPEED);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        if (hasElapsed(2.5))
            return true;
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }
}