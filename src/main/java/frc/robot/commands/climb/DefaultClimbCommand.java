package frc.robot.commands.climb;

import frc.robot.Constants.ClimbConstants;
import frc.robot.OperatorInput;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.LightsSubsystem;

public class DefaultClimbCommand extends LoggingCommand {

    private final ClimbSubsystem  climbSubsystem;
    private final OperatorInput   operatorInput;
    private final LightsSubsystem lightsSubsystem;
    private boolean               climberRaised = false;

    /**
     * Creates a new ExampleCommand.
     *
     * @param climbSubsystem The subsystem used by this command.
     */
    public DefaultClimbCommand(ClimbSubsystem climbSubsystem, OperatorInput operatorInput, LightsSubsystem lightsSubsystem) {
        this.climbSubsystem  = climbSubsystem;
        this.operatorInput   = operatorInput;
        this.lightsSubsystem = lightsSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climbSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (operatorInput.isResetEncoders()) {
            climbSubsystem.resetPrimaryEncoders();
        }
        // This is for climbing
        if (operatorInput.isClimb() > 0.4 && !(operatorInput.isRetract() > 0.4)) {

            climbSubsystem.setMotorSpeeds((ClimbConstants.CLIMBER_MOTOR_SPEED));
        }
        // This is for retracting
        else if (operatorInput.isRetract() > 0.4 && !(operatorInput.isClimb() > 0.4)) {
            climbSubsystem.setMotorSpeeds((ClimbConstants.RETRACT_MOTOR_SPEED));

        }
        else {

            climbSubsystem.stop();
        }
    }



    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }
}