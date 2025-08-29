package frc.robot.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase{

    TalonFX m_driveMotor;
    TalonFX m_turnMotor;

    Slot0Configs m_driveConfig;
    Slot0Configs m_turnConfig;

    MotorOutputConfigs m_driveOutputConfigs;
    MotorOutputConfigs m_turnOutputConfigs;
    
    AnalogEncoder m_absoluteEncoder;

    final PositionVoltage m_turnRequest = new PositionVoltage(0).withSlot(0);
    final VelocityVoltage m_driveRequest = new VelocityVoltage(0).withSlot(0);

    SwerveModuleState m_moduleState;

    SwerveModule(int driveID, int turnID, int absEncoderPort, double absEcoderOffset){
        m_driveMotor = new TalonFX(driveID);
        m_turnMotor = new TalonFX(turnID);

        m_absoluteEncoder = new AnalogEncoder(absEncoderPort);

        m_driveConfig = new Slot0Configs();
        m_turnConfig = new Slot0Configs();

        m_driveConfig.kP = Constants.SwerveConstants.k_driveKP;
        m_driveConfig.kI = Constants.SwerveConstants.k_driveKI;
        m_driveConfig.kD = Constants.SwerveConstants.k_driveKD;

        m_turnConfig.kP = Constants.SwerveConstants.k_turnKP;
        m_turnConfig.kI = Constants.SwerveConstants.k_turnKI;
        m_turnConfig.kD = Constants.SwerveConstants.k_turnKD;

        //TODO I have no clue
        m_turnOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        m_driveOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;


        m_driveOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        m_turnOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        m_driveMotor.getConfigurator().apply(m_driveConfig);
        m_turnMotor.getConfigurator().apply(m_turnConfig);

        m_turnMotor.getConfigurator().setPosition(m_absoluteEncoder.get()-absEcoderOffset);
    }

    public void Drive(SwerveModuleState moduleState){
        m_moduleState = moduleState;
        m_moduleState.optimize(new Rotation2d(m_driveMotor.getPosition().getValueAsDouble()*2*Math.PI));
        m_driveMotor.setControl(m_driveRequest.withVelocity(m_moduleState.speedMetersPerSecond * Constants.SwerveConstants.k_driveGearRatio));
        m_turnMotor.setControl(m_turnRequest.withPosition(m_moduleState.angle.getRotations() * Constants.SwerveConstants.k_turnGearRatio));
    }

    public double getAnglePositionRot(){
        return m_turnMotor.getPosition().getValueAsDouble();
    }

    public double getDrivePositionRot(){
        return m_driveMotor.getPosition().getValueAsDouble();
    }
}
