package frc.robot.commands.coral;


import frc.robot.Constants.CoralConstants;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;


public class CoralCommandEject extends LoggingCommand {


    private CoralSubsystem coralSubsystem;


    /**
     * Creates a new ExampleCommand.
     *
     * @param climbSubsystem The subsystem used by this command.
     */
    public CoralCommandEject(CoralSubsystem coralSubsystem) {


        this.coralSubsystem = coralSubsystem;


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
        coralSubsystem.setMotorSpeeds(CoralConstants.INTAKE_SPEED * 0.3);
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (hasElapsed(2)) {
            return true;
        }
        return false;
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        coralSubsystem.stop();
        logCommandEnd(interrupted);
    }
}

