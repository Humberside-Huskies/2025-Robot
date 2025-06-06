package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.OperatorInput;
import frc.robot.commands.coral.CoralCommandOutake;
import frc.robot.commands.drive.DriveOnHeadingCommand;
import frc.robot.commands.drive.DriveToTargetCommand;
import frc.robot.commands.elevator.SetElevatorLevelCommand;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class AutoCommand extends SequentialCommandGroup {

    public AutoCommand(OperatorInput operatorInput, DriveSubsystem driveSubsystem, CoralSubsystem coralSubsystem,
        ElevatorSubsystem elevatorSubsystem, LightsSubsystem lightsSubsystem, VisionSubsystem visionSubsystem) {

        // Default is to do nothing.
        // If more commands are added, the instant command will end and
        // the next command will be executed.
        addCommands(new InstantCommand());

        AutoPattern autoPattern = operatorInput.getAutoPattern();
        double      autoDelay   = operatorInput.getAutoDelay();

        Alliance    alliance    = DriverStation.getAlliance().orElse(null);

        if (alliance == null) {
            System.out.println("*** ERROR **** unknown Alliance ");
        }

        StringBuilder sb = new StringBuilder();
        sb.append("Auto Selections");
        sb.append("\n   Alliance      : ").append(alliance);
        sb.append("\n   Auto Pattern  : ").append(autoPattern);
        sb.append("\n   Delay         : ").append(autoDelay);

        System.out.println(sb.toString());

        // If any inputs are null, then there was some kind of error.
        if (autoPattern == null) {
            System.out.println("*** ERROR - null found in auto pattern builder ***");
            return;
        }

        /*
         * Delay
         */
        if (autoDelay != 0) {
            addCommands(new WaitCommand(autoDelay));
        }

        /*
         * Compose the appropriate auto commands
         */
        switch (autoPattern) {

        case DO_NOTHING:
            return;

        case DRIVE_FORWARD:

            // Set the current heading to zero, the gyro could have drifted while
            // waiting for auto to start.
            driveSubsystem.setGyroHeading(0);

            // Drive forward 1m at .2 speed
            addCommands(new DriveOnHeadingCommand(0, .2, 100, driveSubsystem));
            return;

        case PATH_TEST_THING:
            addCommands(new DriveToTargetCommand(1, 0, driveSubsystem));
            addCommands(new DriveToTargetCommand(1, 1, driveSubsystem));

            // addCommands(new DriveToTargetCommand(1.5, 0.4, driveSubsystem));

            // addCommands(new DriveToTargetCommand(3, 3, driveSubsystem));
            // addCommands(new DriveToTargetCommand(1, 1, driveSubsystem));

            // double[][] targets = { { 6.26, 4.22 } };

            // for (int i = 0; i < targets.length; i++) {
            // double targetX = targets[i][0];
            // double targetY = targets[i][1];

            // addCommands(new RotateToTargetCommand(targetX, targetY, driveSubsystem));
            // // addCommands(new DriveToTargetCommand(targetX, targetY, driveSubsystem));
            // }
            return;

        case BOX:

            // Set the current heading to zero, the gyro could have drifted while
            // waiting for auto to start.
            driveSubsystem.setGyroHeading(0);

            // Drive out and then one box
            addCommands(new DriveOnHeadingCommand(0, .4, 200, false, driveSubsystem));
            addCommands(new DriveOnHeadingCommand(270, .4, 100, false, driveSubsystem));
            addCommands(new DriveOnHeadingCommand(180, .4, 100, false, driveSubsystem));
            addCommands(new DriveOnHeadingCommand(90, .4, 100, false, driveSubsystem));
            addCommands(new DriveOnHeadingCommand(0, .4, 100, driveSubsystem));
            return;

        case DRIVE_FORWARD_AND_OUTAKE_L1:

            addCommands(new DriveOnHeadingCommand(0, .2, 350, true, driveSubsystem));
            addCommands(new WaitCommand(2.5));
            addCommands(new SetElevatorLevelCommand(ElevatorPosition.CORAL_HEIGHT_L4_ENCODER_COUNT, elevatorSubsystem));
            addCommands(new WaitCommand(3));
            addCommands(new CoralCommandOutake(coralSubsystem));
            addCommands(new WaitCommand(1));
            addCommands(new SetElevatorLevelCommand(ElevatorPosition.CORAL_HEIGHT_L1_ENCODER_COUNT, elevatorSubsystem));
            addCommands(new WaitCommand(1));


            // Added use it with precaution HEADING TO STATION
            // addCommands(new DriveOnHeadingCommand(0, -0.2, 100, true, driveSubsystem));
            // addCommands(new DriveOnHeadingCommand(90, .2, 350, true, driveSubsystem));
            // addCommands(new DriveOnHeadingCommand(0, .2, 500, true, driveSubsystem));

            // Added use with precaution ALIGN to the station for human player
            // addCommands(new DriveOnHeadingCommand(autoDelay, autoDelay, autoDelay, driveSubsystem));

            return;
        }


    }
}