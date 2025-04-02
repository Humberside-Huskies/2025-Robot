// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Input.OperatorInput;
import frc.robot.commands.auto.AutoCommand;
import frc.robot.commands.climb.DefaultClimbCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.commands.elevator.DefaultElevatorCommand;
import frc.robot.commands.vision.DefaultVisionCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LightsSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

        // Subsystems
        // Declarre the lighting subsystem first and pass it into the other subsystem
        // constructors so that they can indicate status information on the lights
        private final LightsSubsystem   lightsSubsystem   = new LightsSubsystem();
        private final DriveSubsystem    driveSubsystem    = new DriveSubsystem(lightsSubsystem);
        private final VisionSubsystem   visionSubsystem   = new VisionSubsystem(lightsSubsystem);
        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(lightsSubsystem);
        private final CoralSubsystem    coralSubsystem    = new CoralSubsystem();
        private final ClimbSubsystem    climbSubsystem    = new ClimbSubsystem(lightsSubsystem);

        // Driver and operator controllers
        private final OperatorInput     operatorInput     = new OperatorInput();

        /** The container for the robot. Contains subsystems, OI devices, and commands. */
        public RobotContainer() {


                // Initialize all Subsystem default commands.
                driveSubsystem.setDefaultCommand(
                        new DefaultDriveCommand(operatorInput, driveSubsystem));

                climbSubsystem.setDefaultCommand(
                        new DefaultClimbCommand(climbSubsystem, operatorInput, lightsSubsystem));

                elevatorSubsystem.setDefaultCommand(
                        new DefaultElevatorCommand(operatorInput, elevatorSubsystem, lightsSubsystem));

                // coralSubsystem.setDefaultCommand(
                // new DefaultCoralCommand(operatorInput, coralSubsystem));
                // Tony is gay
                // Tony is gay

                elevatorSubsystem.setDefaultCommand(
                        new DefaultElevatorCommand(operatorInput, elevatorSubsystem, lightsSubsystem));

                visionSubsystem.setDefaultCommand(
                        new DefaultVisionCommand(driveSubsystem, visionSubsystem));

                // Configure the button bindings - pass in all subsystems
                operatorInput.configureButtonBindings(driveSubsystem, elevatorSubsystem, coralSubsystem, visionSubsystem);

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return new AutoCommand(operatorInput, driveSubsystem, coralSubsystem, elevatorSubsystem, lightsSubsystem,
                        visionSubsystem);
        }
}