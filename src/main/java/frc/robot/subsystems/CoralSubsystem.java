package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {

    // The motors on the left side of the drive.
    private final SparkMax   primaryMotor      = new SparkMax(CoralConstants.CORAL_MOTOR_CAN_ID,
        MotorType.kBrushless);

    private SparkLimitSwitch primaryLimitSwitch;

    private double           primaryMotorSpeed = 0;


    /** Creates a new ElevatorSubsystem. */
    public CoralSubsystem() {

        // right motor
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(CoralConstants.MOTOR_INVERTED)
            .idleMode(IdleMode.kBrake);

        config.limitSwitch.forwardLimitSwitchEnabled(false);
        primaryMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        primaryLimitSwitch = primaryMotor.getForwardLimitSwitch();

    }

    public void setMotorSpeeds(double primaryMotorSpeed) {

        this.primaryMotorSpeed = primaryMotorSpeed;

        primaryMotor.set(this.primaryMotorSpeed);

    }

    public boolean getForwardLimitSwitch() {
        return primaryLimitSwitch.isPressed();
    }

    /** Safely stop the subsystem from moving */
    public void stop() {
        setMotorSpeeds(0);
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("Coral Motor", primaryMotorSpeed);
    }


    @Override
    public String toString() {

        StringBuilder sb = new StringBuilder();

        sb.append(this.getClass().getSimpleName())
            .append(" : left speed").append(Math.round(primaryMotorSpeed * 100.0d) / 100.0d);

        return sb.toString();
    }
}
