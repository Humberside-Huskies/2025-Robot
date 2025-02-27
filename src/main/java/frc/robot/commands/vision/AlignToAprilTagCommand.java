// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class AlignToAprilTagCommand extends LoggingCommand {


    // the target id of the apriltag
    private final int             targetID;

    private final DriveSubsystem  driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final double          targetX     = 0;
    private final PIDController   controllerX = new PIDController(0.03, 0, 0);

    /**
     * DriveForTime command drives at the specified heading at the specified speed for the specified
     * time.
     *
     * @param speed in the range -1.0 to +1.0
     * @param driveSubsystem
     */

    public AlignToAprilTagCommand(int targetID, DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {

        this.targetID        = targetID;
        this.visionSubsystem = visionSubsystem;
        this.driveSubsystem  = driveSubsystem;

        // Add required subsystems
        addRequirements(driveSubsystem);
        controllerX.setTolerance(1);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        logCommandStart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double tx = visionSubsystem.getTX();

        if (visionSubsystem.getTV() == 0) {
            System.out.println("NO TARGETS FOUND");
            driveSubsystem.setMotorSpeeds(0, 0);
            return;
        }


        double turn = controllerX.calculate(tx, targetX);

        // Apply constraints to prevent excessive speed
        turn = Math.max(-0.5, Math.min(turn, 0.5)); // Clamp turn speed

        // setArcadeDriveMotorSpeeds(0, turn, 1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        if (controllerX.atSetpoint())
            return true;
        if (hasElapsed(2.5f)) {
            setFinishReason("Timed out");
            return true;
        }
        return false;
    }
}