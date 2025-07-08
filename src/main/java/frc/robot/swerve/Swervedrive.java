package frc.robot.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swervedrive extends SubsystemBase{
    
    SwerveDriveKinematics m_swerveKinematics;
    Translation2d m_frontLeftPosition;
    Translation2d m_frontRightPosition;
    Translation2d m_backLeftPosition;
    Translation2d m_backRightPosition;


    Swervedrive(){
    }

}
