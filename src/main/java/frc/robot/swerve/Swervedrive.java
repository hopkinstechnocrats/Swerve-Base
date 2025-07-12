package frc.robot.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swervedrive extends SubsystemBase{
    
    SwerveDriveKinematics m_swerveKinematics;
    Translation2d m_frontLeftPosition;
    Translation2d m_frontRightPosition;
    Translation2d m_backLeftPosition;
    Translation2d m_backRightPosition;

    SwerveModule fL;
    SwerveModule fR;
    SwerveModule bL;
    SwerveModule bR;

    SwerveModuleState[] moduleStates;

    Swervedrive(){
        m_frontLeftPosition = new Translation2d(Constants.SwerveConstants.frontLeftX, Constants.SwerveConstants.frontLeftY);
        m_frontRightPosition = new Translation2d(Constants.SwerveConstants.frontRightX, Constants.SwerveConstants.frontRightY);
        m_backLeftPosition = new Translation2d(Constants.SwerveConstants.backLeftX, Constants.SwerveConstants.backLeftY);
        m_backRightPosition = new Translation2d(Constants.SwerveConstants.backRightX, Constants.SwerveConstants.backRightY);

        fL = new SwerveModule(Constants.SwerveConstants.k_frontLeftDriveCANID, Constants.SwerveConstants.k_backLeftTurnCANID);
        fR = new SwerveModule(Constants.SwerveConstants.k_frontRightDriveCANID, Constants.SwerveConstants.k_frontRightTurnCANID);
        bL = new SwerveModule(Constants.SwerveConstants.k_backLeftDriveCANID, Constants.SwerveConstants.k_backLeftTurnCANID);
        bR = new SwerveModule(Constants.SwerveConstants.k_backRightDriveCANID, Constants.SwerveConstants.k_backRightTurnCANID);

        m_swerveKinematics = new SwerveDriveKinematics(m_frontLeftPosition, m_frontRightPosition, m_backLeftPosition, m_backRightPosition);
    }

    public void Drive(ChassisSpeeds desiredState){
        moduleStates = m_swerveKinematics.toSwerveModuleStates(desiredState);
        fL.Drive(moduleStates[0]);
        fR.Drive(moduleStates[1]);
        bL.Drive(moduleStates[2]);
        bR.Drive(moduleStates[3]);
    }
}
