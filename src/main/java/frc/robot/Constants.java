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

        public static final int    DRIVER_CONTROLLER_PORT     = 0;
        public static final double DRIVER_CONTROLLER_DEADBAND = .2;
    }

    public static final class AutoConstants {

        public static enum AutoPattern {
            DO_NOTHING, DRIVE_FORWARD, BOX, PATH_TEST_THING, MY_CASE;

        }

        public static RobotConfig config;
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
        public static final double  DRIVE_SCALING_NORMAL = .6;
        public static final double  DRIVE_SCALING_SLOW   = .3;
    }

    public static final class LightsConstants {

        public static final int LED_STRING_PWM_PORT = 0;
        public static final int LED_STRING_LENGTH   = 60;
    }

    public static final class ElevatorConstant {
        public static final int    ELEVATOR_MOTOR_CAN_ID  = 0;
        public static final double ELEVATOR_CLIMB_SPEED   = .5;
        public static final double ELEVATOR_RETRACT_SPEED = -.5;
    }

    public static final class CoralConstant {
        public static final double CORAL_MOTOR_INTAKE_SPEED  = .5;
        public static final double CORAL_MOTOR_OUTTAKE_SPEED = .5;
        public static final int    RIGHT_CORAL_MOTOR_CAN_ID  = 12;
        public static final int    LEFT_CORAL_MOTOR_CAN_ID   = 10;

    }

    public static final class VisionConstant {
        public static final double AMBIGUITY_THRESHOLD_MEGATAG = 0;
        public static final double mountedAngleDegrees         = 18.0;
        public static final double mountedHeightMeters         = 0.160655;

        // distance from the target to the floor
        public static final double StationHeightMeters         = 1.49;
        public static final double ReefHeightMeters            = 0.308;
        public static final double ProcessorHeightMeters       = 1.301;
        public static final double BargeHeightMeters           = 1.868;
    }
}
