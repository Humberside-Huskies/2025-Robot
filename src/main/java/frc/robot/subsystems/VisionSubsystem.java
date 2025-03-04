package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private NetworkTable limelightTable;
    private double       tv, ty, tx, ta, tid, ambiguity;
    private double[]     botPose;

    public VisionSubsystem(LightsSubsystem lightsSubsystem) {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getTX() {
        return tx;
    }

    public double getTY() {
        return ty;
    }

    public double getTV() {
        return tv;
    }

    public double getTA() {
        return ta;
    }

    // id of apriltag
    public double getTID() {
        return tid;
    }

    public Pose2d getBotPose() {
        return new Pose2d(new Translation2d(botPose[0], botPose[1]), new Rotation2d().fromDegrees(botPose[5]));
    }

    public double getAmbiguity() {
        return ambiguity;
    }


    /** Safely stop the subsystem from moving */
    public void stop() {
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        tv        = limelightTable.getEntry("tv").getDouble(0);
        tx        = limelightTable.getEntry("tx").getDouble(0);
        ty        = limelightTable.getEntry("ty").getDouble(0);
        ta        = limelightTable.getEntry("ta").getDouble(0);

        botPose   = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        ambiguity = limelightTable.getEntry("rawFiducials").getDoubleArray(new double[7])[6];
        System.out.println(Arrays.toString(limelightTable.getEntry("rawFiducials").getDoubleArray(new double[8])));
        System.out.println(Arrays.toString(limelightTable.getKeys().toArray()));

        // System.out.println(getBotPose().getX() + ", " + getBotPose().getY());

    }

    @Override
    public String toString() {
        // Create an appropriate text readable string describing the state of the
        // subsystem

        return "Vlad created the file, tony is gay";
    }
}