package frc.robot.commands.algae;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.OperatorInput;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.AlgaeSubsystem;

public class DefaultAlgaeCommand extends LoggingCommand {

    private final AlgaeSubsystem algaeSubsystem;
    private final OperatorInput  operatorInput;

    /**
     * Creates a new DefaultDriveCommand.
     *
     * @param operatorInput which contains the drive mode selector.
     * @param algaeSubsystem The subsystem used by this command.
     */
    public DefaultAlgaeCommand(OperatorInput operatorInput, AlgaeSubsystem algaeSubsystem) {

        this.operatorInput  = operatorInput;
        this.algaeSubsystem = algaeSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(algaeSubsystem);
    }

    @Override
    public void initialize() {
        logCommandStart();
    }

    @Override
    public void execute() {

        double algaeJoystick = operatorInput.algaeJoystick();
        double intakeSpeed   = operatorInput.getAlgaeMotorSpeed();

        if (algaeSubsystem.getSensor() && intakeSpeed == 0) {
            intakeSpeed = AlgaeConstants.INTAKE_HOLD_ALGAE_SPEED;
        }

        algaeSubsystem.setArmMotorSpeed(algaeJoystick);
        algaeSubsystem.setIntakeMotorSpeed(intakeSpeed);

    }

    @Override
    public boolean isFinished() {
        return false; // default commands never end but can be interrupted
    }

    @Override
    public void end(boolean interrupted) {

        logCommandEnd(interrupted);
    }



}