package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstant;

public class ElevatorSubsystem extends SubsystemBase {

    /**
     * NavX - AHRS
     *
     * This local NavXGryro is used to override the value in the gyro dashboard sendable to use
     * {@link AHRS#getAngle()} which includes any offset, instead of the {@link AHRS#getYaw()} which
     * is the raw yaw value without the offset.
     * <p>
     * Using the getAngle() method makes the gyro appear in the correct position on the dashboard
     * accounting for the offset.
     */

    private final LightsSubsystem lightsSubsystem;

    // The motors on the left side of the drive.
    private final SparkMax        elevatorMotor   = new SparkMax(ElevatorConstant.ELEVATOR_MOTOR_CAN_ID,
        MotorType.kBrushless);

    private double                elevatorSpeed   = 0;

    // Encoders
    private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();


    /** Creates a new ElevatorSubsystem. */
    public ElevatorSubsystem(LightsSubsystem lightsSubsystem) {

        this.lightsSubsystem = lightsSubsystem;

        /*
         * Configure Left Side Motors
         */
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false)
            .idleMode(IdleMode.kBrake)
            .disableFollowerMode();
        elevatorMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        resetElevatorEncoders();
    }



    public double getEncoderDistanceCm() {

        return getEncoder() * DriveConstants.CM_PER_ENCODER_COUNT;
    }


    /**
     * Gets the left velocity.
     *
     * @return the left drive encoder speed
     */
    public double getEncoderSpeed() {
        return elevatorEncoder.getVelocity();
    }



    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public double getEncoder() {
        return elevatorMotor.getEncoder().getPosition();
    }

    /** Resets the drive encoders to zero. */
    public void resetElevatorEncoders() {

        // Reset the offsets so that the encoders are zeroed.
        elevatorMotor.getEncoder().setPosition(0);
    }

    public void setMotorSpeeds(double elevatorSpeed) {

        this.elevatorSpeed = elevatorSpeed;

        elevatorMotor.set(this.elevatorSpeed);
    }

    /** Safely stop the subsystem from moving */
    public void stop() {
        setMotorSpeeds(0);
    }

    @Override
    public void periodic() {


        SmartDashboard.putNumber("Right Motor", elevatorSpeed);

        SmartDashboard.putNumber("Encoder", Math.round(getEncoder() * 100) / 100d);
        SmartDashboard.putNumber("Distance (cm)", Math.round(getEncoderDistanceCm() * 10) / 10d);
        SmartDashboard.putNumber("Velocity", Math.round(getEncoderSpeed() * 100) / 100d);


    }


    @Override
    public String toString() {

        StringBuilder sb = new StringBuilder();

        sb.append(this.getClass().getSimpleName()).append(" : ")
            .append(", Drive dist ").append(Math.round(getEncoderDistanceCm() * 10) / 10d).append("cm");

        return sb.toString();
    }
}
