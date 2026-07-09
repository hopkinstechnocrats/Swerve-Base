package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants{

    public static final class SwerveConstants{
        public static final int k_frontLeftDriveCANID = 13;
        public static final int k_frontRightDriveCANID = 12;
        public static final int k_backLeftDriveCANID = 11;
        public static final int k_backRightDriveCANID = 10;

        public static final int k_frontLeftTurnCANID = 6;
        public static final int k_frontRightTurnCANID = 7;
        public static final int k_backLeftTurnCANID = 8;
        public static final int k_backRightTurnCANID = 9;

        public static final int k_flAbsEncoderPort = 22;
        public static final int k_frAbsEncoderPort = 21;
        public static final int k_blAbsEncoderPort = 23;
        public static final int k_brAbsEncoderPort = 20;

        public static final double k_driveKP = 0.3;
        public static final double k_driveKI = 0.0;
        public static final double k_driveKD = 0.0;
        public static final double k_driveKV = 2.55;

        public static final double k_turnKP = 12.0;
        public static final double k_turnKI = 5.0;
        public static final double k_turnKD = 0.0;

        public static final double frontLeftX = 0.21761;
        public static final double frontLeftY = 0.31921;
        public static final double frontRightX = 0.21761;
        public static final double frontRightY = -0.31921;
        public static final double backLeftX = -0.21761;
        public static final double backLeftY = 0.31921;
        public static final double backRightX = -0.21761;
        public static final double backRightY = -0.31921;

        public static final double k_flAbsEncoderOffset = -0.171;
        public static final double k_frAbsEncoderOffset = -0.989;
        public static final double k_blAbsEncoderOffset = 0.325;
        public static final double k_brAbsEncoderOffset = 0.221;

        public static final double k_maxLinearSpeedMeterPerSecond = 4.3;
        public static final double k_slowSpeed = 1.8;
        public static final double k_midSpeed = 2.57;
        public static final double k_maxAngularSpeedRadPerSec = 10.0 * Math.PI;

        public static final double k_driveGearRatio = 6.03;
        public static final double k_turnGearRatio = 287/11;
        public static final double k_wheelCircumferenceMeters = 0.1016 * Math.PI;

        public static final Pose2d k_startPose = new Pose2d(0, 0, new Rotation2d(0));
    }
 
    public static final class ControlConstants{
        public static final double k_driveControllerDeadband = 0.1;
        public static final double k_operatorControllerDeadband = 0.1;
        public static final int k_driverPort = 0;
        public static final int k_operatorPort = 1;
    }
    
    public static final class GyroConstants{
        public static final int k_gyroID = 15;
    }


}
