// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose.
 * <p>
 * All constants should be declared globally (i.e. public static).
 * <br>
 * Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double DEFAULT_COMMAND_TIMEOUT_SECONDS = 5;

    public static final class OperatorInputConstants {

        public static final int    DRIVER_CONTROLLER_PORT       = 0;
        public static final double DRIVER_CONTROLLER_DEADBAND   = .2;

        public static final int    OPERATOR_CONTROLLER_PORT     = 1;
        public static final double OPERATOR_CONTROLLER_DEADBAND = 0.2;
    }

    public static final class AutoConstants {

        public static enum AutoPattern {
            DO_NOTHING, DRIVE_FORWARD, BOX, PATH_TEST_THING, DRIVE_FORWARD_AND_OUTAKE_L1;

        }

        public static RobotConfig config;
    }

    public static final class ClimbConstants {
        public static final int    PRIMARY_MOTOR_PORT  = 50;

        public static final double CLIMBER_MOTOR_SPEED = 0.5;
        public static final double RETRACT_MOTOR_SPEED = -0.65;
    }

    public static final class DriveConstants {

        public static enum DriveMode {
            TANK, ARCADE, SINGLE_STICK_LEFT, SINGLE_STICK_RIGHT;
        }

        // NOTE: Follower motors are at CAN_ID+1
        public static final int     LEFT_MOTOR_CAN_ID    = 10;
        public static final int     RIGHT_MOTOR_CAN_ID   = 20;

        public static final boolean LEFT_MOTOR_INVERTED  = false;
        public static final boolean RIGHT_MOTOR_INVERTED = true;

        public static final double  CM_PER_ENCODER_COUNT = 3.503;

        public static final boolean GYRO_INVERTED        = false;

        /** Proportional gain for gyro pid tracking */
        public static final double  GYRO_PID_KP          = 0.01;

        public static final double  DRIVE_SCALING_BOOST  = 1;
        public static final double  DRIVE_SCALING_NORMAL = .2;
        public static final double  DRIVE_SCALING_SLOW   = .3;

        public static final double  ROBOT_WIDTH          = .6;
    }

    public static final class LightsConstants {

        public static final int LED_STRING_PWM_PORT = 0;
        public static final int LED_STRING_LENGTH   = 60;
    }

    public static final class ElevatorConstants {
        public static final int    ELEVATOR_MOTOR_CAN_ID         = 30;
        public static final double ELEVATOR_CONTRACT_SPEED       = .5;
        public static final double ELEVATOR_RETRACT_SPEED        = -.5;

        public static final double ELEVATOR_TOLERANCE            = 2.5;

        public static final double ELEVATOR_SLOW_ZONE_SPEED      = .2;
        public static final double ELEVATOR_MAX_SPEED            = .5;

        public static final double ENCODER_COUNTS_PER_REVOLUTION = 16.8;
        public static final double ROBOT_WHEEL_DIAMETER_CM       = 6 * 2.54;
        public static final double CM_PER_ENCODER_COUNT          = (ROBOT_WHEEL_DIAMETER_CM * Math.PI)
            / ENCODER_COUNTS_PER_REVOLUTION;
    }

    public static final class CoralConstants {
        public static final int     CORAL_MOTOR_CAN_ID      = 50;
        public static final boolean MOTOR_INVERTED          = false;
        public static final double  INTAKE_SPEED            = 0.5;

        public static final double  CORAL_HEIGHT_LEVEL_1_CM = 0;
        public static final double  CORAL_HEIGHT_LEVEL_2_CM = 0;
        public static final double  CORAL_HEIGHT_LEVEL_3_CM = 181;
        public static final double  CORAL_HEIGHT_LEVEL_4_CM = 47.5;
    }

    public static final class VisionConstants {
        public static final double AMBIGUITY_THRESHOLD_MEGATAG = 0.3;
        public static final double mountedAngleDegrees         = 18.0;
        public static final double mountedHeightMeters         = 0.160655;

        // distance from the target to the floor
        public static final double StationHeightMeters         = 1.49;
        public static final double ReefHeightMeters            = 0.308;
        public static final double ProcessorHeightMeters       = 1.301;
        public static final double BargeHeightMeters           = 1.868;
    }
}
