package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class AlgaeSubsystem extends SubsystemBase {

    private final SparkMax armMotor         = new SparkMax(AlgaeConstants.ARM_MOTOR_CAN_ID,
        MotorType.kBrushless);

    private final SparkMax intakeMotor      = new SparkMax(
        AlgaeConstants.INTAKE_MOTOR_CAN_ID,
        MotorType.kBrushless);

    private double         armMotorSpeed    = 0;
    private double         intakeMotorSpeed = 0;


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
        armMotor.set(this.armMotorSpeed);
    }


    public void setIntakeMotorSpeed(double intakeMotorSpeed) {

        this.intakeMotorSpeed = intakeMotorSpeed;
        intakeMotor.set(this.intakeMotorSpeed);
    }


    /**
     * Gets the arm encoder.
     *
     */
    public double getArmEncoder() {
        return armMotor.getEncoder().getPosition();
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
    }


    @Override
    public String toString() {

        StringBuilder sb = new StringBuilder();
        // TODO: do the string

        return sb.toString();
    }
}
// A - init position
// B - intake position
// X - shoot position
// Y - remove position
// RT - Out