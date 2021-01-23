/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Swerve {
        public static final double kWheelRadius = 0.0508;
        public static final int kEncoderResolution = 4096;
    
        public static final double kModuleMaxAngularVelocity = Math.PI;
        public static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared
        public static final double kTwoPi = 2 * Math.PI;

        public static final double MaxSpeedOfWheel = 5.0;
    }

    public static final class Drivetrain {
        public static final double maxMetersPerSecond = 3.66;
        public static final double maxRadiansPerSecond = 8.76;
        public static final int frontLeftDriveMotorPort = 8;
        public static final int frontLeftTurnMotorPort = 1;
        public static final int rearLeftDriveMotorPort = 2;
        public static final int rearLeftTurnMotorPort = 3;
        public static final int frontRightDriveMotorPort = 4;
        public static final int frontRightTurnMotorPort = 5;
        public static final int rearRightDriveMotorPort = 6;
        public static final int rearRightturnMotorPort = 7;
        public static final int frontLeftAbsoluteEncoder = 0;
        public static final int frontRightAbsoluteEncoder = 1;
        public static final int rearLeftAbsoluteEncoder = 3;
        public static final int rearRightAbsoluteEncoder = 2;

        public static final double gyroRateDeadzone = 0.05;
    }

    public static final class Physical {
        /*
         * When using these values for the swerve drive, make sure to divide them by two
         * if the pivot point of the bot is in the center.
         * 
         * Note that standard convention is the bot is centered on the origin, facing
         * right, along the positive x-axis. So the top-left portion of the bot is in
         * the first quadrant, the back-left is in the second quadrant, the back-right
         * is in the third quadrant, and the back-right is in the fourth quadrant.
         */

        // Note: these are the distances of the swerve wheels to each other, not the
        // size of the bot's frame.
        public static final double widthInMeters = .67;
        public static final double lengthInMeters = .5;
        public static final double weightInKgs = 0; // TODO this isn't the real weight!!!
        public static final double staticFrictionConstant = .0205;

    }

}
