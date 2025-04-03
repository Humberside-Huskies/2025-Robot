package frc.robot.commands.algae;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AlgaeConstants.AlgaeArmRotation;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaeIntakeCommand extends LoggingCommand {
    private final AlgaeSubsystem algaeSubsystem;
    private double               error;
    private Timer                timer = new Timer();

    /**
     * Creates a new ExampleCommand.
     *
     * @param climbSubsystem The subsystem used by this command.
     */
    public AlgaeIntakeCommand(AlgaeSubsystem algaeSubsystem) {
        this.algaeSubsystem = algaeSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(algaeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
        timer.reset();
        timer.stop();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // testing
        // algaeSubsystem.setIntakeMotorSpeed(0.1);

        error = AlgaeConstants.AlgaeArmRotation.ALGAE_INTAKE_Angle.getDegrees() - algaeSubsystem.getArmDegrees();

        if (Math.abs(error) < AlgaeConstants.HOLD_TOLERANCE)
            algaeSubsystem.setArmMotorSpeed(0);


        else if (Math.abs(error) < AlgaeConstants.ARM_SLOW_TOLERANCE) {
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

        algaeSubsystem.setIntakeMotorSpeed(AlgaeConstants.INTAKE_SPEED);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (algaeSubsystem.getSensor())
            timer.start();
        if (algaeSubsystem.getSensor() && timer.hasElapsed(.3))
            return true;
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        algaeSubsystem.setIntakeMotorSpeed(0);
        algaeSubsystem.setArmMotorSpeed(0);
        if (algaeSubsystem.getSensor() && !interrupted)
            CommandScheduler.getInstance().schedule(new SetAlgaeCommand(AlgaeArmRotation.ALGAE_RESET_Angle, algaeSubsystem));
        logCommandEnd(interrupted);
    }
}
