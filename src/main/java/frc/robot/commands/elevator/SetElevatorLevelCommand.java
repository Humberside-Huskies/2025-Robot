package frc.robot.commands.elevator;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorLevelCommand extends LoggingCommand {

    private ElevatorSubsystem   elevatorSubsystem;

    private double              targetHeightCm;
    private final PIDController encoderController = new PIDController(0.05, 0, 0);

    /**
     * Creates a new ExampleCommand.
     *
     * @param climbSubsystem The subsystem used by this command.
     */
    public SetElevatorLevelCommand(double targetHeightCm, ElevatorSubsystem elevatorSubsystem) {
        this.targetHeightCm    = targetHeightCm;
        this.elevatorSubsystem = elevatorSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(elevatorSubsystem);
        encoderController.setTolerance(5);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        logCommandStart();
        elevatorSubsystem.resetPrimaryEncoders();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double speed = encoderController.calculate(elevatorSubsystem.getEncoderDistanceCm(), targetHeightCm);
        elevatorSubsystem.setMotorSpeeds(speed);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (encoderController.atSetpoint())
            return true;
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }
}