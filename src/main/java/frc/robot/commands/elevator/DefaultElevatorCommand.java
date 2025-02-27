package frc.robot.commands.elevator;

import frc.robot.Constants.ElevatorConstant;
import frc.robot.OperatorInput;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LightsSubsystem;

public class DefaultElevatorCommand extends LoggingCommand {

    private ElevatorSubsystem elevatorSubsystem;
    private OperatorInput     operatorInput;
    private LightsSubsystem   lightsSubsystem;
    private boolean           climberRaised = false;

    /**
     * Creates a new ExampleCommand.
     *
     * @param climbSubsystem The subsystem used by this command.
     */
    public DefaultElevatorCommand(OperatorInput operatorInput, ElevatorSubsystem elevatorSubsystem,
        LightsSubsystem lightsSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.operatorInput     = operatorInput;
        this.lightsSubsystem   = lightsSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(elevatorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
        elevatorSubsystem.resetElevatorEncoders();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (operatorInput.isClimb() > 0.4 && !(operatorInput.isRetract() > 0.4)) {

            elevatorSubsystem.setMotorSpeeds(ElevatorConstant.ELEVATOR_CLIMB_SPEED);
        }
        else if (operatorInput.isRetract() > 0.4 && !(operatorInput.isClimb() > 0.4)) {
            elevatorSubsystem.setMotorSpeeds(ElevatorConstant.ELEVATOR_RETRACT_SPEED);
        }
        else {

            elevatorSubsystem.stop();
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