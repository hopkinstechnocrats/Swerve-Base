package frc.robot.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase{

    TalonFX m_driveMotor;
    TalonFX m_turnMotor;

    Slot0Configs m_driveConfig;
    Slot0Configs m_turnConfig;
    
    final PositionVoltage m_turnRequest = new PositionVoltage(0).withSlot(0);
    final VelocityVoltage m_driveRequest = new VelocityVoltage(0).withSlot(0);

    SwerveModule(int driveID, int turnID){
        m_driveMotor = new TalonFX(driveID);
        m_turnMotor = new TalonFX(turnID);

        m_driveConfig = new Slot0Configs();
        m_turnConfig = new Slot0Configs();

        m_driveConfig.kP = Constants.SwerveConstants.k_driveKP;
        m_driveConfig.kI = Constants.SwerveConstants.k_driveKI;
        m_driveConfig.kD = Constants.SwerveConstants.k_driveKD;

        m_turnConfig.kP = Constants.SwerveConstants.k_turnKP;
        m_turnConfig.kI = Constants.SwerveConstants.k_turnKI;
        m_turnConfig.kD = Constants.SwerveConstants.k_turnKD;

        m_driveMotor.getConfigurator().apply(m_driveConfig);
        m_turnMotor.getConfigurator().apply(m_turnConfig);
    }

    public void DrivePID(double velocityRotPerSec){
        m_driveMotor.setControl(m_driveRequest.withVelocity(velocityRotPerSec));
    }

    public void TurnPID(double positionRotations){
        m_turnMotor.setControl(m_turnRequest.withPosition(positionRotations));
    }
}
