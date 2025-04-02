package frc.robot.commands.coral;

import frc.robot.Input.OperatorInput;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.CoralSubsystem;

public class DefaultCoralCommand extends LoggingCommand {

    private CoralSubsystem coralSubsystem;
    private boolean        isIntake;
    private OperatorInput  operatorInput;

    /**
     * Creates a new ExampleCommand.
     *
     * @param climbSubsystem The subsystem used by this command.
     */
    public DefaultCoralCommand(OperatorInput operatorInput, CoralSubsystem coralSubsystem) {

        this.operatorInput  = operatorInput;
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
    // @Override
    // public void execute() {
    // double coralJoystick = operatorInput.CoralJoystick();

    // Pushing
    // if (coralJoystick > 0) {
    // coralSubsystem.setMotorSpeeds(-CoralConstants.INTAKE_SPEED * Math.abs(coralJoystick));
    // }
    // Pulling
    // else if (coralJoystick < 0) {
    // coralSubsystem.setMotorSpeeds(CoralConstants.INTAKE_SPEED * Math.abs(coralJoystick));
    // }
    // }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        coralSubsystem.stop();
        logCommandEnd(interrupted);
    }
}