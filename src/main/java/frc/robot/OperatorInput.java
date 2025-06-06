package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AlgaeConstants.AlgaeArmRotation;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.Constants.OperatorInputConstants;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.GameController;
import frc.robot.commands.algae.AlgaeIntakeCommand;
import frc.robot.commands.algae.SetAlgaeCommand;
import frc.robot.commands.coral.CoralCommandEject;
import frc.robot.commands.coral.CoralCommandIntake;
import frc.robot.commands.coral.CoralCommandOutake;
import frc.robot.commands.elevator.SetElevatorLevelCommand;
import frc.robot.subsystems.AlgaeSubsystem;
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
    SendableChooser<AutoPattern> autoPatternChooser = new SendableChooser<>();
    SendableChooser<Integer>     waitTimeChooser    = new SendableChooser<>();
    SendableChooser<DriveMode>   driveModeChooser   = new SendableChooser<>();

    /**
     * Construct an OperatorInput class that is fed by a DriverController and optionally an
     * OperatorController.
     */
    public OperatorInput() {

        driverController   = new GameController(OperatorInputConstants.DRIVER_CONTROLLER_PORT,
            OperatorInputConstants.DRIVER_CONTROLLER_DEADBAND);
        operatorController = new GameController(OperatorInputConstants.OPERATOR_CONTROLLER_PORT,
            OperatorInputConstants.OPERATOR_CONTROLLER_DEADBAND);

        // Initialize the dashboard selectors
        autoPatternChooser.setDefaultOption("Do Nothing", AutoPattern.DO_NOTHING);
        SmartDashboard.putData("Auto Pattern", autoPatternChooser);
        autoPatternChooser.addOption("Drive Forward", AutoPattern.DRIVE_FORWARD);
        autoPatternChooser.addOption("Box", AutoPattern.BOX);
        autoPatternChooser.addOption("Path Test", AutoPattern.PATH_TEST_THING);
        autoPatternChooser.addOption("Actual Auto", AutoPattern.DRIVE_FORWARD_AND_OUTAKE_L1);

        waitTimeChooser.setDefaultOption("No wait", 0);
        SmartDashboard.putData("Auto Wait Time", waitTimeChooser);
        waitTimeChooser.addOption("1 second", 1);
        waitTimeChooser.addOption("3 seconds", 3);
        waitTimeChooser.addOption("5 seconds", 5);

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
        ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem,
        AlgaeSubsystem algaeSubsystem) {

        // Cancel Command - cancels all running commands on all subsystems
        new Trigger(() -> isCancel())
            .onTrue(new CancelCommand(this, driveSubsystem, elevatorSubsystem));

        new Trigger(() -> operatorController.getStartButton())
            .onTrue(new CancelCommand(this, driveSubsystem, elevatorSubsystem));

        // new Trigger(() -> driveToAprilTag())
        // .onTrue(new AlignToAprilTagCommand(driveSubsystem, visionSubsystem));

        // new Trigger(() -> driverController.getPOV() == 90)
        // .onTrue(new AlignToReefCommand(driveSubsystem, visionSubsystem, ReefOffsetAngle.Right));

        // new Trigger(() -> driverController.getPOV() == 270)
        // .onTrue(new AlignToReefCommand(driveSubsystem, visionSubsystem, ReefOffsetAngle.Left));

        // Operator Control
        new Trigger(() -> operatorController.getYButton())
            .onTrue(new SetElevatorLevelCommand(ElevatorPosition.CORAL_HEIGHT_L1_ENCODER_COUNT, elevatorSubsystem));

        new Trigger(() -> operatorController.getBButton())
            .onTrue(new SetElevatorLevelCommand(ElevatorPosition.CORAL_HEIGHT_L2_ENCODER_COUNT, elevatorSubsystem));

        new Trigger(() -> operatorController.getAButton())
            .onTrue(new SetElevatorLevelCommand(ElevatorPosition.CORAL_HEIGHT_L3_ENCODER_COUNT, elevatorSubsystem));

        new Trigger(() -> operatorController.getXButton())
            .onTrue(new SetElevatorLevelCommand(ElevatorPosition.CORAL_HEIGHT_L4_ENCODER_COUNT, elevatorSubsystem));

        // Coral Shooter
        new Trigger(() -> isIntake())
            .onTrue(new CoralCommandIntake(coralSubsystem));

        new Trigger(() -> isOutTake())
            .onTrue(new CoralCommandOutake(coralSubsystem));

        new Trigger(() -> isEject())
            .onTrue(new CoralCommandEject(coralSubsystem));

        // Algae
        new Trigger(() -> driverController.getXButton())
            .onTrue(new SetAlgaeCommand(AlgaeArmRotation.ALGAE_RESET_Angle, algaeSubsystem));

        new Trigger(() -> driverController.getBButton())
            .onTrue(new SetAlgaeCommand(AlgaeArmRotation.ALGAE_OUTTAKE_Angle, algaeSubsystem));

        new Trigger(() -> driverController.getYButton())
            .onTrue(new SetAlgaeCommand(AlgaeArmRotation.ALGAE_REMOVE_Angle, algaeSubsystem));


        new Trigger(() -> driverController.getAButton())
            .onTrue(new AlgaeIntakeCommand(algaeSubsystem));

    }

    /*
     * Auto Pattern Selectors
     */
    public AutoPattern getAutoPattern() {
        return autoPatternChooser.getSelected();
    }

    public Integer getAutoDelay() {
        return waitTimeChooser.getSelected();
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

    /*
     * This section deals with coral intake and outake system
     */

    public boolean isIntake() {
        return operatorController.getLeftTriggerAxis() > 0;
    }

    public boolean isOutTake() {
        return operatorController.getRightTriggerAxis() > 0;
    }

    public boolean isEject() {
        return operatorController.getLeftBumperButton();
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

    @Override
    public void periodic() {
        SmartDashboard.putString("Driver Controller", driverController.toString());
    }

    public double CoralJoystick() {
        double value = operatorController.getRightY();

        if (Math.abs(value) < 0.1) {
            return 0;
        }
        return value;
    }

    public double ElevatorJoystick() {
        double value = operatorController.getLeftY();

        if (Math.abs(value) < 0.1) {
            return 0;
        }
        return value;
    }

    public boolean isResetEncoders() {
        return driverController.getBackButton();
    }

    public double getAlgaeMotorSpeed() {

        if (driverController.getLeftTriggerAxis() > 0) {
            return -AlgaeConstants.INTAKE_SPEED;
        }
        if (driverController.getRightTriggerAxis() > 0) {
            return AlgaeConstants.INTAKE_SPEED;
        }
        return 0;
    }

    public double algaeJoystick() {
        double avalue = operatorController.getRightY();

        if (Math.abs(avalue) < 0.1) {
            return 0;
        }
        return avalue * AlgaeConstants.ARM_SPEED_FAST;
    }
}
