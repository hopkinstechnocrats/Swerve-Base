package frc.robot.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase{

    TalonFX m_driveMotor;
    TalonFX m_turnMotor;

    TalonFXConfiguration m_driveConfig;
    TalonFXConfiguration m_turnConfig;

    CANcoder m_absoluteEncoder;

    CANcoderConfiguration m_encoderConfig;

    final PositionVoltage m_turnRequest = new PositionVoltage(0).withSlot(0);
    final VelocityVoltage m_driveRequest = new VelocityVoltage(0).withSlot(0);

    SwerveModuleState m_moduleState;

    SwerveModule(int driveID, int turnID, int absEncoderPort, double absEcoderOffset){
        //GertrudeGreyser is the name of our canivore, change this if on rio canbus
        m_driveMotor = new TalonFX(driveID, new CANBus("GertrudeGreyser"));
        m_turnMotor = new TalonFX(turnID, new CANBus("GertrudeGreyser"));

        m_absoluteEncoder = new CANcoder(absEncoderPort, new CANBus("GertrudeGreyser"));

        //create config before modifying
        m_encoderConfig = new CANcoderConfiguration();

        m_encoderConfig.MagnetSensor.MagnetOffset = absEcoderOffset;

        m_driveConfig = new TalonFXConfiguration();
        m_turnConfig = new TalonFXConfiguration();

        m_driveConfig.Slot0.kP = Constants.SwerveConstants.k_driveKP;
        m_driveConfig.Slot0.kI = Constants.SwerveConstants.k_driveKI;
        m_driveConfig.Slot0.kD = Constants.SwerveConstants.k_driveKD;
        //kV is feed forwards value, this takes care of most of your driving
        m_driveConfig.Slot0.kV = Constants.SwerveConstants.k_driveKV;

        m_turnConfig.Slot0.kP = Constants.SwerveConstants.k_turnKP;
        m_turnConfig.Slot0.kI = Constants.SwerveConstants.k_turnKI;
        m_turnConfig.Slot0.kD = Constants.SwerveConstants.k_turnKD;

        //makes the motor use the cancoder as its main source of position data
        m_turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        m_turnConfig.Feedback.RotorToSensorRatio = Constants.SwerveConstants.k_turnGearRatio;
        m_turnConfig.Feedback.FeedbackRemoteSensorID = m_absoluteEncoder.getDeviceID();

        m_driveConfig.Feedback.SensorToMechanismRatio = Constants.SwerveConstants.k_driveGearRatio;

        //makes it so that turn motor sees 0 degrees as the same value as 360
        m_turnConfig.ClosedLoopGeneral.ContinuousWrap = true;

        m_turnConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        m_driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        m_turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //makes it so the drive can't pull too many amps, effectively a ramp rate
        m_driveConfig.CurrentLimits.StatorCurrentLimit = 80;

        //apply all the configs
        m_driveMotor.getConfigurator().apply(m_driveConfig);
        m_turnMotor.getConfigurator().apply(m_turnConfig);
        m_absoluteEncoder.getConfigurator().apply(m_encoderConfig);
    }


    public void Drive(SwerveModuleState moduleState){
        m_moduleState = moduleState;
        //optimizing makes it so that it always turns the smallest amount possible
        m_moduleState.optimize(this.getAngleRotation2d());
        //cosine compensation, this accounts for the rotation so that we continue moving in a straight line without drifting when rotating
        m_moduleState.speedMetersPerSecond *= m_moduleState.angle.minus(this.getAngleRotation2d()).getCos();
        m_driveMotor.setControl(m_driveRequest.withVelocity(m_moduleState.speedMetersPerSecond));
        m_turnMotor.setControl(m_turnRequest.withPosition(m_moduleState.angle.getRotations()));
    }

    public double getAnglePositionRot(){
        return m_absoluteEncoder.getPosition().getValueAsDouble();
    }

    public double getDrivePositionRot(){
        return m_driveMotor.getPosition().getValueAsDouble();
    }

    public double getDriveDistanceMeters(){
        return this.getDrivePositionRot()*Constants.SwerveConstants.k_wheelCircumferenceMeters;
    }

    public Rotation2d getAngleRotation2d(){
        return new Rotation2d(m_absoluteEncoder.getPosition().getValueAsDouble() * Math.PI * 2); 
    }

    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(this.getDriveDistanceMeters(), this.getAngleRotation2d());
    }

    public double getAbsEncoderPositionRot(){
        return m_absoluteEncoder.getPosition().getValueAsDouble();
    }

    public double getDriveVelocityMeterPerSec(){
        return m_driveMotor.getVelocity().getValueAsDouble() * Constants.SwerveConstants.k_wheelCircumferenceMeters;
    }
}
