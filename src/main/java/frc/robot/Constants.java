// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final static boolean atCompetition = false;

    public final class UniversalConstants {
        // robot weight 131.6 lbs
        public final static double gravityMetersPerSecondSquared = 9.81;
        public final static double defaultPeriodSeconds = 0.02;

        public final static String canivoreName = "CTRENetwork2";

        public enum Direction {
            left,
            right
        }

        public static final double frameWidthMeters = Units.inchesToMeters(27);

        public static final double bumperWidthMeters = Units.inchesToMeters(27 + 7);

    }

    public final static class ControllerConstants {
        public static final double controllerDeadzone = 0.06;
        public static final double maxThrottle = 1.0;
    }

    public final static class DrivetrainConstants {
        // KINEMATICS CONSTANTS

        /**
         * Distance between the center point of the left wheels and the center point of the right wheels.
         */
        public static final double trackwidthMeters = Units.inchesToMeters(22.0);
        /**
         * Distance between the center point of the front wheels and the center point of the back wheels.
         */
        public static final double wheelbaseMeters = Units.inchesToMeters(22.0);
        /**
         * Distance from the center of the robot to each swerve module.
         */
        public static final double drivetrainRadiusMeters = Math.hypot(wheelbaseMeters / 2.0, trackwidthMeters / 2.0); //0.4177


        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
            new Translation2d(wheelbaseMeters / 2.0, -trackwidthMeters / 2.0),
            new Translation2d(-wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
            new Translation2d(-wheelbaseMeters / 2.0, -trackwidthMeters / 2.0)
        );

        public static final double frameWidthMeters = Units.inchesToMeters(27.0);

        public static final double bumperWidthMeters = Units.inchesToMeters(27 + 7);
        public static final double halfBumperWidthMeters = bumperWidthMeters / 2.0;



        /**
         * The maximum possible velocity of the robot in meters per second.
         * <br>
         * This is a measure of how fast the robot will be able to drive in a straight line, based off of the empirical free speed of the drive Krakens.
         */
        // public static final double maxAchievableVelocityMetersPerSecond = 1.0;
        public static final double krakenFreeSpeedRPM = 5800;
        public static final double krakenFreeSpeedRotationsPerSecond = krakenFreeSpeedRPM / 60.;
        public static final double maxAchievableVelocityMetersPerSecond = krakenFreeSpeedRotationsPerSecond *
            (SwerveModuleConstants.driveGearReduction) * SwerveModuleConstants.wheelCircumferenceMeters; // ~5.23 using a theoretical wheel radius of 2 inches m/s
                                                                                                       // ~5.06 when adding 1/16 of an inch of wheel sink into the carpet.
                                                                                                       // ~5.10 using an emperical measurement of wheel radius on fresh wheels.
                                                                                                       // Actual top speed based on testing is ~4.7 m/s
                                                                                                       // (calculating top speed using kv yeilds [12 / 2.42] ~ 4.96 m/s,
                                                                                                       //  but I don't think we can actually achieve this because 
                                                                                                       //  the battery voltage will likely drop below 12 when all drive motors are running)
                                                                                                       // To give ourselves a little breathing room, we use a max speed of 4.5 m/s in auto.

        /**
         * This is the max desired speed that will be achievable in teleop.
         * <br>
         * If the controller joystick is maxed in one direction, it will drive at this speed.
         * <br>
         * This value will be less than or equal to the maxAchievableVelocityMetersPerSecond, depending on driver preference.
         */
        public static final double maxDesiredTeleopVelocityMetersPerSecond = maxAchievableVelocityMetersPerSecond; 

        /**
         * The maximum achievable angular velocity of the robot in radians per second.
         * <br>
         * This is a measure of how fast the robot can rotate in place, based off of maxAchievableVelocityMetersPerSecond.
         */

        public static final double maxAchievableAngularVelocityRadiansPerSecond = maxAchievableVelocityMetersPerSecond / drivetrainRadiusMeters; // Theoretical ~1.93 rotations per second
                                                                                                                                                 // using 4.7 m/s for max linear speed yeilds ~1.79 rotations per second
                                                                                                                                                 // using 4.5 m/s for max linear speed yeilds ~1.71 rotations per second
                                                                                                                                                 // we use 1.0 rotations per second in auto to be extra conservative
                                                                                                                                                 // because any time you're rotating, you're taking away from your translational speed.

        /**
         * This is the max desired angular velocity that will be achievable in teleop.
         * <br>
         * If the controller rotation joystick is maxed in one direction, it will rotate at this speed.
         * <br>
         * This value will be tuned based off of driver preference.
         */ 
        // 1.7 rot per sec is about our max
        public static final double maxDesiredTeleopAngularVelocityRadiansPerSecond = Units.rotationsToRadians(1.5);

    }


    public final static class SwerveModuleConstants {
        /** Rotations of the drive wheel per rotations of the drive motor. */
        // public static final double driveGearReduction = (50.0 / 15.0) * (17.0 / 27.0) * (45.0 / 15.0);

        // /** Rotations of the steering column per rotations of the angle motor. */
        // public static final double steerGearReduction = (50.0 / 14.0) * (60.0 / 10.0);

        // The wheels have a 2 inch radius, but sink into the capet about (1/16) of an inch.
        // As an estimate, the wheel radius is Units.inchesToMeters(2.-1./16.), or 0.0492m
        // public static final double wheelRadiusMeters = 0.04946; //use MeasureWheelDiameter for this!
        // public static final double wheelCircumferenceMeters = 2 * Math.PI * wheelRadiusMeters; // ~0.31

                /** Rotations of the drive wheel per rotations of the drive motor. */
        public static final double driveGearReductionSIM = (15.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

        // max torque ratio
        public static final double driveGearReduction = (12.0 / 54.0) * (32.0 / 25.0) * (15.0 / 30.0);

        // public static final double driveGearReduction = (14.0 / 54.0) * (32.0 / 25.0) * (15.0 / 30.0);

        /** Rotations of the steering column per rotations of the angle motor. */
        public static final double steerGearReduction = (14.0 / 50.0) * (10.0 / 60.0);

        // The wheels have a 2 inch radius, but sink into the capet about (1/16) of an inch.
        // As an estimate, the wheel radius is Units.inchesToMeters(2.-1./16.), or 0.0492m
        // public static final double wheelRadiusMeters = 0.04946; //use MeasureWheelDiameter for this!
        public static final double wheelRadiusMeters = 0.05019730723396923;// use to be nits.inchesToMeters(2.015434249374315)
        public static final double wheelCircumferenceMeters = 2 * Math.PI * wheelRadiusMeters; // ~0.31

        //0.05128784124270502
        //0.0511920299341076
        // PID + FEEDFORWARD CONSTANTS FOR MOTORS
        // PID for drive motors.
        public static final double drivekPVoltsPerMeterPerSecond = 0;
        public static final double drivekIVoltsPerMeter = 0.;
        public static final double drivekDVoltsPerMeterPerSecondSquared = 0.;

        // PID for angle motors.
        public static final double anglekPVoltsPerDegree = 0.08;
        public static final double anglekIVoltsPerDegreeSeconds = 0.; // this might be the wrong unit idk 
        public static final double anglekDVoltsPerDegreePerSecond = 0.;

        public static final double drivekSVolts = 0.2383;
        public static final double drivekVVoltsSecondsPerMeter = 2.52;
        public static final double drivekAVoltsSecondsSquaredPerMeter = 0.;
        
        // Motor configs
        public static final int angleContinuousCurrentLimit = 50;
        public static final boolean angleInvert = true;
        
        public static final int driveContinuousCurrentLimit = 60;
        public static final boolean driveInvert = true;
    }


    public final static class GyroConstants {
        public static final int pigeonID = 50;


        //Follow the mount calibration process in Phoenix Tuner to determine these
        public static final double mountPoseYawDegrees = -90.45111083984375;
        public static final double mountPosePitchDegrees = -16.21670913696289;
        public static final double mountPoseRollDegrees = 89.97844696044922;
    }

    public final static class VisionConstants {
        //Camera, IP, hostname
        //http://10.17.87.12:5800 this is for front/BW2 and fuel/C2     
        //http://10.17.87.11:5800/ this is for left/BW3, right/BW4, and back/BW1                                           

        public final static Transform3d robotToFront = new Transform3d(
            new Translation3d(0.258, -0.003, 0.527),
            new Rotation3d(-Math.toRadians(0.118), -Math.toRadians(25.142), Math.toRadians(0.525))
        );// 10.5 forward in 0.5 in to right/left, 21.5 in height

        public final static Transform3d robotToLeft = new Transform3d(
            new Translation3d(-0.015, 0.256, 0.533),
            new Rotation3d(-Math.toRadians(1.9), -Math.toRadians(22.5), Math.toRadians(94.0))
        );

        public final static Transform3d robotToRight = new Transform3d(
            new Translation3d(0.005, -0.262, 0.536),
            new Rotation3d(-Math.toRadians(0.22), -Math.toRadians(24.3), -Math.toRadians(89.7))
        );

        public final static Transform3d robotToBack = new Transform3d(
            new Translation3d(-0.262, 0.002, 0.53),
            new Rotation3d(-Math.toRadians(0.18), -Math.toRadians(24.9), Math.toRadians(179.9))
        );

        public final static Transform3d robotToFuelCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(-9.75), Units.inchesToMeters(5.5), Units.inchesToMeters(26.)),
            new Rotation3d(0, Math.toRadians(19), Math.toRadians(-12))
        );

        public final static String[] tagCameraNames = {
            "front",
            "back",
            "left",
            "right"
        };

        public final static Transform3d[] tagCameraTransforms = {
            robotToFront,
            robotToBack,
            robotToLeft,
            robotToRight
        };

    }

    public final static class LEDConstants {
        public final static int ledPWMPort = 0;

        //total number of leds
        public final static int ledsPerStrip = 60;

        public final static double stripLengthMeters = 1.0;

        public final static double ledsPerMeter = (1.0 * ledsPerStrip) / stripLengthMeters;

        public final static double metersPerLed = 1/ledsPerMeter;

        /**
         * Hues for specific colors
         * Values use the openCV convention where hue ranges from [0, 180)
         */
        public final static class Hues {

            public final static int orangeSignalLight = 4;
            public final static int blueBumpers = 114;
            public final static int redBumpers = 0;
            public final static int redTrafficLight = 0;//0;
            public final static int greenTrafficLight = 40;//60;
            public final static int betweenBlueAndRed = 150; // a purple/pink that's between blue and red.

        }
    }

}
