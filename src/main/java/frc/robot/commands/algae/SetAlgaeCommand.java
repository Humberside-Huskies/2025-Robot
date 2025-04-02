package frc.robot.commands.algae;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AlgaeConstants.AlgaeArmRotation;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.LightsSubsystem;

public class SetAlgaeCommand extends LoggingCommand {
    private final AlgaeSubsystem  algaeSubsystem;
    private final LightsSubsystem lightsSubsystem;

    private AlgaeArmRotation      targetDegreeCounts;
    private double                error;

    /**
     * Creates a new ExampleCommand.
     *
     * @param climbSubsystem The subsystem used by this command.
     */
    public SetAlgaeCommand(AlgaeSubsystem algaeSubsystem, LightsSubsystem lightsSubsystem) {
        this.algaeSubsystem  = algaeSubsystem;
        this.lightsSubsystem = lightsSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(algaeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // testing
        // algaeSubsystem.setIntakeMotorSpeed(0.1);

        error = targetDegreeCounts.getDegrees() * AlgaeConstants.gearRatio - algaeSubsystem.getArmEncoder();

        if (Math.abs(error) < AlgaeConstants.ARM_SLOW_TOLERANCE) {
            if (error > 0)
                algaeSubsystem.setArmMotorSpeed(AlgaeConstants.ARM_SPEED_SLOW);
            else
                algaeSubsystem.setArmMotorSpeed(-AlgaeConstants.ARM_SPEED_SLOW);
        }
        else {
            if (error > 0)
                algaeSubsystem.setArmMotorSpeed(AlgaeConstants.ARM_SPEED_FAST);
            else
                algaeSubsystem.setArmMotorSpeed(-AlgaeConstants.ARM_SPEED_FAST);
        }

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (Math.abs(error) < AlgaeConstants.HOLD_TOLERANCE)
            return true;
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }
}