package frc.robot.commands.elevator;

import frc.robot.Input.OperatorInput;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LightsSubsystem;

public class DefaultElevatorCommand extends LoggingCommand {

    private ElevatorSubsystem elevatorSubsystem;
    private OperatorInput     operatorInput;
    private LightsSubsystem   lightsSubsystem;

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

    <<<<<<<

    Updated upstream


    =======>>>>>>>Stashed changes

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double elevatorJoystick = operatorInput.ElevatorJoystick();

<<<<<<< Updated upstream

        // Climbing
        if (elevatorJoystick > 0) {
            elevatorSubsystem.setMotorSpeeds(ElevatorConstants.ELEVATOR_CONTRACT_SPEED * Math.abs(elevatorJoystick));
            elevatorSubsystem.manual_elevator_on = true;
=======
        // Climbing
        if (elevatorJoystick > 0) {
            elevatorSubsystem.setMotorSpeeds(ElevatorConstants.ELEVATOR_CONTRACT_SPEED * Math.abs(elevatorJoystick));
>>>>>>> Stashed changes
        }
        // Retracting
        else if (elevatorJoystick < 0) {
            elevatorSubsystem.setMotorSpeeds(ElevatorConstants.ELEVATOR_RETRACT_SPEED * Math.abs(elevatorJoystick));
<<<<<<< Updated upstream
            elevatorSubsystem.manual_elevator_on = true;
=======
>>>>>>> Stashed changes
        }
        else {
            if (elevatorSubsystem.getEncoder() > 10)
                elevatorSubsystem.setMotorSpeeds(ElevatorConstants.HOLD_SPEED);
            else
                elevatorSubsystem.stop();
        }
    }


<<<<<<< Updated upstream

=======
>>>>>>> Stashed changes
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