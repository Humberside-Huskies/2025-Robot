package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.OperatorInputConstants;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.GameController;
import frc.robot.commands.coral.CoralCommandIntake;
import frc.robot.commands.coral.CoralCommandOutake;
import frc.robot.commands.elevator.SetElevatorLevelCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * The DriverController exposes all driver functions
 * <p>
 * Extend SubsystemBase in order to have a built in periodic call to support SmartDashboard updates
 */
public class OperatorInput extends SubsystemBase {

    private final GameController driverController;
    private final GameController operatorController;

    // Auto Setup Choosers
    SendableChooser<DriveMode>   driveModeChooser = new SendableChooser<>();
    SendableChooser<Command>     autoChooser      = new SendableChooser<>();

    /**
     * Construct an OperatorInput class that is fed by a DriverController and optionally an
     * OperatorController.
     */
    public OperatorInput() {

        driverController   = new GameController(OperatorInputConstants.DRIVER_CONTROLLER_PORT,
            OperatorInputConstants.DRIVER_CONTROLLER_DEADBAND);
        operatorController = new GameController(OperatorInputConstants.OPERATOR_CONTROLLER_PORT,
            OperatorInputConstants.OPERATOR_CONTROLLER_DEADBAND);

        driveModeChooser.setDefaultOption("Arcade", DriveMode.ARCADE);
        SmartDashboard.putData("Drive Mode", driveModeChooser);
        driveModeChooser.addOption("Tank", DriveMode.TANK);
        driveModeChooser.addOption("Single Stick (L)", DriveMode.SINGLE_STICK_LEFT);
        driveModeChooser.addOption("Single Stick (R)", DriveMode.SINGLE_STICK_RIGHT);
    }

    /**
     * Configure the button bindings for all operator commands
     * <p>
     * NOTE: This routine requires all subsystems to be passed in
     * <p>
     * NOTE: This routine must only be called once from the RobotContainer
     *
     * @param driveSubsystem
     */
    public void configureButtonBindings(DriveSubsystem driveSubsystem,
        ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem) {

        // Cancel Command - cancels all running commands on all subsystems
        new Trigger(() -> isCancel())
            .onTrue(new CancelCommand(this, driveSubsystem, elevatorSubsystem));

        new Trigger(() -> operatorController.getStartButton())
            .onTrue(new CancelCommand(this, driveSubsystem, elevatorSubsystem));

        // new Trigger(() -> driveToAprilTag())
        // .onTrue(new AlignToAprilTagCommand(driveSubsystem, visionSubsystem));

        // Coral Shooter
        new Trigger(() -> isIntake())
            .onTrue(new CoralCommandIntake(coralSubsystem));

        new Trigger(() -> isOutTake())
            .onTrue(new CoralCommandOutake(coralSubsystem));



        // Elevator Level setter
        // Configure the DPAD to drive one meter on a heading
        new Trigger(() -> operatorController.getPOV() == 0)
            .onTrue(new SetElevatorLevelCommand(ElevatorPosition.CORAL_HEIGHT_L1_ENCODER_COUNT, elevatorSubsystem));

        new Trigger(() -> operatorController.getPOV() == 90)
            .onTrue(new SetElevatorLevelCommand(ElevatorPosition.CORAL_HEIGHT_L2_ENCODER_COUNT, elevatorSubsystem));

        new Trigger(() -> operatorController.getPOV() == 180)
            .onTrue(new SetElevatorLevelCommand(ElevatorPosition.CORAL_HEIGHT_L3_ENCODER_COUNT, elevatorSubsystem));

        new Trigger(() -> operatorController.getPOV() == 270)
            .onTrue(new SetElevatorLevelCommand(ElevatorPosition.CORAL_HEIGHT_L4_ENCODER_COUNT, elevatorSubsystem));

        registerAuto(driveSubsystem, elevatorSubsystem, coralSubsystem);
        autoChooser.setDefaultOption("TopAuto", driveSubsystem.getPPAutoCommand("TopAuto"));
        autoChooser.setDefaultOption("Test", driveSubsystem.getPPAutoCommand("Test"));
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    private void registerAuto(DriveSubsystem driveSubsystem, ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem) {

        NamedCommands.registerCommand("elevator1",
            new SetElevatorLevelCommand(ElevatorPosition.CORAL_HEIGHT_L1_ENCODER_COUNT, elevatorSubsystem));

        NamedCommands.registerCommand("elevator2",
            new SetElevatorLevelCommand(ElevatorPosition.CORAL_HEIGHT_L2_ENCODER_COUNT, elevatorSubsystem));

        NamedCommands.registerCommand("elevator3",
            new SetElevatorLevelCommand(ElevatorPosition.CORAL_HEIGHT_L3_ENCODER_COUNT, elevatorSubsystem));

        NamedCommands.registerCommand("shoot", new CoralCommandOutake(coralSubsystem));
        NamedCommands.registerCommand("intake", new CoralCommandIntake(coralSubsystem));
    }

    public Command getAutoCommand() {
        return autoChooser.getSelected();
    }

    /*
     * Cancel Command support
     * Do not end the command while the button is pressed
     */
    public boolean isCancel() {
        return driverController.getStartButton();
    }

    /*
     * The following routines are used by the default commands for each subsystem
     *
     * They allow the default commands to get user input to manually move the
     * robot elements.
     */
    /*
     * Drive Subsystem
     */
    public DriveMode getSelectedDriveMode() {
        return driveModeChooser.getSelected();
    }

    public boolean isBoost() {
        return driverController.getLeftBumperButton();
    }

    public boolean isSlowDown() {
        return driverController.getRightBumperButton();
    }

    public double getLeftSpeed() {
        return driverController.getLeftY();
    }

    public double getRightSpeed() {
        return driverController.getRightY();
    }

    public double getSpeed() {

        if (driveModeChooser.getSelected() == DriveMode.SINGLE_STICK_RIGHT) {
            return driverController.getRightY();
        }

        return driverController.getLeftY();
    }

    public double getTurn() {

        if (driveModeChooser.getSelected() == DriveMode.SINGLE_STICK_LEFT) {
            return driverController.getLeftX();
        }

        return driverController.getRightX();
    }

    public boolean isIntake() {
        return operatorController.getAButton();
    }

    public boolean isOutTake() {
        return operatorController.getBButton();
    }

    /*
     * Support for haptic feedback to the driver
     */
    public void startVibrate() {
        driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
    }

    public void stopVibrate() {
        driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }

    // public boolean driveToAprilTag() {
    // return driverController.getAButton();
    // }


    @Override
    public void periodic() {
        SmartDashboard.putString("Driver Controller", driverController.toString());
    }

    public double CoralJoystick() {
        double value = operatorController.getRightY();

        if (Math.abs(value) < 0.1)
            return 0;
        return value;
    }

    public double ElevatorJoystick() {
        double value = operatorController.getLeftY();

        if (Math.abs(value) < 0.1)
            return 0;
        return value;
    }

    // public double isElevatorRetract() {
    // double value = operatorController.getRightTriggerAxis();

    // if (value < 0.1)
    // return 0;
    // return value;
    // }

    public boolean isResetEncoders() {
        return driverController.getBackButton();
    }

    /*
     * Is Climber Trigger Button
     */
    public double isClimb() {
        return driverController.getLeftTriggerAxis();
    }

    // cool function buddy climbing things
    public double isRetract() {
        return driverController.getRightTriggerAxis();
    }



    // public boolean

}
