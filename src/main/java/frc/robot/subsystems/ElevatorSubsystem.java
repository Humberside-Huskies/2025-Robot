package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

    private final LightsSubsystem lightsSubsystem;

    // The motors on the left side of the drive.
    private final SparkMax        primaryMotor   = new SparkMax(ElevatorConstants.ELEVATOR_MOTOR_CAN_ID,
        MotorType.kBrushless);

    private DigitalInput          lowerLimit     = new DigitalInput(0);

    private double                primarySpeed   = 0;

    // Encoders
    private final RelativeEncoder primaryEncoder = primaryMotor.getEncoder();


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
        primaryMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetPrimaryEncoders();
    }

    public double getEncoderDistanceCm() {
        return getEncoder() * ElevatorConstants.CM_PER_ENCODER_COUNT + ElevatorConstants.DIST_FROM_GROUND_CM;
    }

    public boolean atLowerLimit() {
        return !lowerLimit.get();
    }

    /**
     * Gets the left velocity.
     *
     * @return the left drive encoder speed
     */
    public double getEncoderSpeed() {
        return primaryEncoder.getVelocity();
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public double getEncoder() {
        return primaryMotor.getEncoder().getPosition();
    }

    /** Resets the drive encoders to zero. */
    public void resetPrimaryEncoders() {

        // Reset the offsets so that the encoders are zeroed.
        primaryMotor.getEncoder().setPosition(0);
    }

    public void setMotorSpeeds(double primarySpeed) {

        if (atLowerLimit() && primarySpeed < 0) {
            primarySpeed = 0;
            resetPrimaryEncoders();
        }

        this.primarySpeed = primarySpeed;
        primaryMotor.set(this.primarySpeed);
    }

    /** Safely stop the subsystem from moving */
    public void stop() {
        setMotorSpeeds(0);
    }

    @Override
    public void periodic() {


        SmartDashboard.putNumber("Right Motor", primarySpeed);

        SmartDashboard.putNumber("Elevator Encoder", Math.round(getEncoder() * 100) / 100d);
        SmartDashboard.putNumber("Distance (cm)", Math.round(getEncoderDistanceCm() * 10) / 10d);
        SmartDashboard.putNumber("Velocity", Math.round(getEncoderSpeed() * 100) / 100d);
        SmartDashboard.putBoolean("Lower Limit", atLowerLimit());

    }


    @Override
    public String toString() {

        StringBuilder sb = new StringBuilder();

        sb.append(this.getClass().getSimpleName()).append(" : ")
            .append(", Drive dist ").append(Math.round(getEncoderDistanceCm() * 10) / 10d).append("cm");

        return sb.toString();
    }
}
