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
        public static final int    PRIMARY_MOTOR_PORT  = 40;

        public static final double CLIMBER_MOTOR_SPEED = 0.5;
        public static final double RETRACT_MOTOR_SPEED = -0.65;
    }

    public static final class DriveConstants {

        public static enum DriveMode {
            TANK, ARCADE, SINGLE_STICK_LEFT, SINGLE_STICK_RIGHT;
        }

        // NOTE: Follower motors are at CAN_ID+1
        public static final int     LEFT_MOTOR_CAN_ID             = 20;
        public static final int     RIGHT_MOTOR_CAN_ID            = 10;

        public static final boolean LEFT_MOTOR_INVERTED           = false;
        public static final boolean RIGHT_MOTOR_INVERTED          = true;

        public static final double  ENCODER_COUNTS_PER_REVOLUTION = 8.46;
        public static final double  ROBOT_WHEEL_DIAMETER_CM       = 15.24;
        public static final double  CM_PER_ENCODER_COUNT          = (ROBOT_WHEEL_DIAMETER_CM * Math.PI) * 2
            / ENCODER_COUNTS_PER_REVOLUTION;

        public static final boolean GYRO_INVERTED                 = false;

        /** Proportional gain for gyro pid tracking */
        public static final double  GYRO_PID_KP                   = 0.01;

        public static final double  DRIVE_SCALING_BOOST           = 1;
        public static final double  DRIVE_SCALING_NORMAL          = .5;
        public static final double  DRIVE_SCALING_SLOW            = .3;

        public static final double  ROBOT_WIDTH                   = .6;
    }

    public static final class LightsConstants {

        public static final int LED_STRING_PWM_PORT = 0;
        public static final int LED_STRING_LENGTH   = 60;
    }

    public static final class ElevatorConstants {
        public static final int    ELEVATOR_MOTOR_CAN_ID         = 30;
        public static final double ELEVATOR_CONTRACT_SPEED       = .3;
        public static final double ELEVATOR_RETRACT_SPEED        = -.2;

        public static final double ELEVATOR_TOLERANCE            = 2.5;

        public static final double ELEVATOR_SLOW_ZONE_SPEED      = .2;
        public static final double ELEVATOR_MAX_SPEED            = .5;
        public static final double DIST_FROM_GROUND_CM           = 41.5;

        public static final double ENCODER_COUNTS_PER_REVOLUTION = 19.7;
        public static final double ROBOT_WHEEL_DIAMETER_CM       = 15;
        public static final double CM_PER_ENCODER_COUNT          = (ROBOT_WHEEL_DIAMETER_CM * Math.PI) * 2
            / ENCODER_COUNTS_PER_REVOLUTION;

        public static final double HOLD_SPEED                    = 0.05;
        public static final double SLOW_SPEED                    = 0.07;
        public static final double HOLD_TOLERANCE                = 5;
        public static final double SLOW_TOLERANCE                = 10;
        public static final double FAST_SPEED                    = 0.55;

        public static enum ElevatorPosition {
            CORAL_HEIGHT_L1_ENCODER_COUNT(
                3),
            CORAL_HEIGHT_L2_ENCODER_COUNT(
                40.00),
            CORAL_HEIGHT_L3_ENCODER_COUNT(
                66.00),
            CORAL_HEIGHT_L4_ENCODER_COUNT(113.17);

            private final double counts;


            private ElevatorPosition(double counts) {
                this.counts = counts;
            }


            public double getEncoderCounts() {
                return counts;
            }
        };

    }

    public static final class CoralConstants {
        public static final int     CORAL_MOTOR_CAN_ID = 50;
        public static final boolean MOTOR_INVERTED     = false;
        public static final double  INTAKE_SPEED       = 0.2;
        public static final double  OUTTAKE_SPEED      = 0.3;

        public static enum ReefOffsetAngle {

            RIGHT_CORAL(
                15),
            LEFT_CORAL(
                -15);

            private final double offset;

            private ReefOffsetAngle(double offset) {
                this.offset = offset;
            }

            public double getOffset() {
                return offset;

            }
        }
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

    public static final class AlgaeConstants {

        public static final int     ARM_MOTOR_CAN_ID                    = 60;
        public static final int     INTAKE_MOTOR_CAN_ID                 = 61;

        public static final boolean ARM_MOTOR_INVERTED                  = true;
        public static final boolean INTAKE_MOTOR_INVERTED               = false;

        // Algae arm rotation speed
        public static final double  ARM_SPEED_SLOW                      = .2;
        public static final double  ARM_SPEED_FAST                      = .3;
        public static final double  ARM_HOLD_SPEED                      = -.02;
        public static final double  ARM_HOLD_ALGAE_SPEED_GAIN           = -.01;

        // Algae Intake Motor speed
        public static final double  INTAKE_SPEED                        = 0.3;
        public static final double  OUTTAKE_SPEED                       = -0.3;

        // Algae arm's speed tolerance
        public static final double  ARM_SLOW_TOLERANCE                  = 5;

        public static final double  HOLD_TOLERANCE                      = 3;

        public static final double  GEAR_RATIO_DEGREE_PER_ENCODER_COUNT = 360 / 72;


        public static enum AlgaeArmRotation {
            ALGAE_INTAKE_Angle(
                95),
            ALGAE_OUTTAKE_Angle(
                35),
            ALGAE_REMOVE_Angle(
                29),
            ALGAE_RESET_Angle(
                10);

            private final double counts;


            private AlgaeArmRotation(double counts) {
                this.counts = counts;
            }


            public double getDegrees() {
                return counts;
            }
        };
    }
}
