package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {

    private final SparkMax     armMotor         = new SparkMax(AlgaeConstants.ARM_MOTOR_CAN_ID,
        MotorType.kBrushless);

    private final SparkMax     intakeMotor      = new SparkMax(
        AlgaeConstants.INTAKE_MOTOR_CAN_ID,
        MotorType.kBrushless);

    private final DigitalInput sensor           = new DigitalInput(1);

    private double             armMotorSpeed    = 0;
    private double             intakeMotorSpeed = 0;
    private double             OutakeMotorSpeed = 0;


    /** Creates a new AlgaeSubsystem. */
    public AlgaeSubsystem() {

        // motor setup
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(AlgaeConstants.INTAKE_MOTOR_INVERTED)
            .idleMode(IdleMode.kBrake);

        intakeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config = new SparkMaxConfig();
        config.inverted(AlgaeConstants.ARM_MOTOR_INVERTED)
            .idleMode(IdleMode.kBrake);

        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setArmMotorSpeed(double armMotorSpeed) {

        this.armMotorSpeed = armMotorSpeed;

        armMotor.set(this.armMotorSpeed + getHoldCurrent());
    }


    public void setIntakeMotorSpeed(double intakeMotorSpeed) {

        this.intakeMotorSpeed = intakeMotorSpeed;
        intakeMotor.set(this.intakeMotorSpeed);
    }



    public void setOutakeMotorSpeed(double OutakeMotorSpeed) {

        this.OutakeMotorSpeed = OutakeMotorSpeed;
        intakeMotor.set(this.OutakeMotorSpeed);
    }

    public boolean getSensor() {
        return !sensor.get();
    }

    /**
     * Gets the arm encoder.
     *
     */
    public double getArmEncoder() {
        return armMotor.getEncoder().getPosition();
    }

    public double getArmDegrees() {
        return getArmEncoder() * AlgaeConstants.GEAR_RATIO_DEGREE_PER_ENCODER_COUNT;
    }

    public double getHoldCurrent() {
        double holdCurrent = AlgaeConstants.ARM_HOLD_SPEED;

        if (getSensor())
            holdCurrent += AlgaeConstants.ARM_HOLD_ALGAE_SPEED_GAIN;
        return Math.sin(getArmDegrees() * (Math.PI / 180)) * holdCurrent;
    }

    /** Safely stop the subsystem from moving */
    public void stop() {
        setArmMotorSpeed(0);
        setIntakeMotorSpeed(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Motor", Math.round(armMotorSpeed * 100.0d) / 100.0d);
        SmartDashboard.putNumber("Intake Motor", Math.round(intakeMotorSpeed * 100.0d) / 100.0d);
        SmartDashboard.putNumber("Arm Angle", getArmDegrees());
        SmartDashboard.putNumber("Arm Encoder", getArmEncoder());
        SmartDashboard.putBoolean("Algae Sensor", getSensor());
    }


    @Override
    public String toString() {

        StringBuilder sb = new StringBuilder();
        // TODO: do the string

        return sb.toString();
    }
}