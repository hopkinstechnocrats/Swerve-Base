package frc.robot.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.networktables.DoubleEntry;
import frc.robot.Constants;

public class Swervedrive extends SubsystemBase{
    
    SwerveDriveKinematics m_swerveKinematics;
    Translation2d m_frontLeftPosition;
    Translation2d m_frontRightPosition;
    Translation2d m_backLeftPosition;
    Translation2d m_backRightPosition;

    Pose2d m_pose;
    SwerveDrivePoseEstimator m_poseEstimator;
    
    NetworkTableInstance inst;
    NetworkTable table;

    StructArrayPublisher<SwerveModuleState> desiredStatePublisher;
    StructArrayPublisher<SwerveModuleState> actualStatePublisher;

    StructPublisher<Pose2d> robotPosition;

    DoubleEntry flAnalog;
    DoubleEntry frAnalog;
    DoubleEntry blAnalog;
    DoubleEntry brAnalog;

    SwerveModule fL;
    SwerveModule fR;
    SwerveModule bL;
    SwerveModule bR;

    Gyro gyro;

    //desired states are the commanded value from controller, actual states are the states as reported by the robot
    SwerveModuleState[] desiredModuleStates;
    SwerveModuleState[] actualModuleState = {new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};

    ChassisSpeeds m_speeds;

    public Swervedrive(){
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("Swerve");

        desiredStatePublisher = table.getStructArrayTopic("Desired Module States", SwerveModuleState.struct).publish();
        actualStatePublisher = table.getStructArrayTopic("Actual Module States", SwerveModuleState.struct).publish();

        m_frontLeftPosition = new Translation2d(Constants.SwerveConstants.frontLeftX, Constants.SwerveConstants.frontLeftY);
        m_frontRightPosition = new Translation2d(Constants.SwerveConstants.frontRightX, Constants.SwerveConstants.frontRightY);
        m_backLeftPosition = new Translation2d(Constants.SwerveConstants.backLeftX, Constants.SwerveConstants.backLeftY);
        m_backRightPosition = new Translation2d(Constants.SwerveConstants.backRightX, Constants.SwerveConstants.backRightY);

        fL = new SwerveModule(Constants.SwerveConstants.k_frontLeftDriveCANID, Constants.SwerveConstants.k_frontLeftTurnCANID, 
                Constants.SwerveConstants.k_flAbsEncoderPort, Constants.SwerveConstants.k_flAbsEncoderOffset);
        fR = new SwerveModule(Constants.SwerveConstants.k_frontRightDriveCANID, Constants.SwerveConstants.k_frontRightTurnCANID, 
                Constants.SwerveConstants.k_frAbsEncoderPort, Constants.SwerveConstants.k_frAbsEncoderOffset);
        bL = new SwerveModule(Constants.SwerveConstants.k_backLeftDriveCANID, Constants.SwerveConstants.k_backLeftTurnCANID,
                Constants.SwerveConstants.k_blAbsEncoderPort, Constants.SwerveConstants.k_blAbsEncoderOffset);
        bR = new SwerveModule(Constants.SwerveConstants.k_backRightDriveCANID, Constants.SwerveConstants.k_backRightTurnCANID,
                Constants.SwerveConstants.k_brAbsEncoderPort, Constants.SwerveConstants.k_brAbsEncoderOffset);

        m_swerveKinematics = new SwerveDriveKinematics(m_frontLeftPosition, m_frontRightPosition, m_backLeftPosition, m_backRightPosition);

        gyro = new Gyro(Constants.GyroConstants.k_gyroID);

        //pose estimator is our main way of keeping track of where the robot on the field, start position is a constant 2d field location
        m_poseEstimator = new SwerveDrivePoseEstimator(m_swerveKinematics, gyro.getRotation(), new SwerveModulePosition[]{
            fL.getModulePosition(), fR.getModulePosition(), bL.getModulePosition(), bR.getModulePosition()
        }, Constants.SwerveConstants.k_startPose);

        flAnalog = table.getDoubleTopic("FL Absolute Encoder").getEntry(0);
        frAnalog = table.getDoubleTopic("FR Absolute Encoder").getEntry(0);
        blAnalog = table.getDoubleTopic("BL Absolute Encoder").getEntry(0);
        brAnalog = table.getDoubleTopic("BR Absolute Encoder").getEntry(0);

        robotPosition = table.getStructTopic("Robot Position", Pose2d.struct).publish();
    }


    @Override
    public void periodic(){
        //updates our robot position with rotation as tracked by the gyro and using the change in position from wheels, can also use vision readings (see 2026 code)
        m_pose = m_poseEstimator.update(gyro.getRotation(), new SwerveModulePosition[]{
             fL.getModulePosition(), fR.getModulePosition(), bL.getModulePosition(), bR.getModulePosition()
        });

        //update all out network table values
        desiredStatePublisher.set(desiredModuleStates);

        this.updateActualStates();

        actualStatePublisher.set(actualModuleState);

        flAnalog.set(fL.getAbsEncoderPositionRot());
        frAnalog.set(fR.getAbsEncoderPositionRot());
        blAnalog.set(bL.getAbsEncoderPositionRot());
        brAnalog.set(bR.getAbsEncoderPositionRot());
        robotPosition.set(m_pose);
    }

    //our main function for driving
    public void Drive(ChassisSpeeds desiredState){
        //breaks apart a chassis speed into the needed states of all four modules in an array
        desiredModuleStates = m_swerveKinematics.toSwerveModuleStates(desiredState);
        // fl = 0, fr = 1, bl = 2, br = 3
        fL.Drive(desiredModuleStates[0]);
        fR.Drive(desiredModuleStates[1]);
        bL.Drive(desiredModuleStates[2]);
        bR.Drive(desiredModuleStates[3]);
    }

    
    private void updateActualStates(){
        actualModuleState[0] = new SwerveModuleState(fL.getDriveVelocityMeterPerSec(), fL.getAngleRotation2d());
        actualModuleState[1] = new SwerveModuleState(fR.getDriveVelocityMeterPerSec(), fR.getAngleRotation2d());
        actualModuleState[2] = new SwerveModuleState(bL.getDriveVelocityMeterPerSec(), bL.getAngleRotation2d());
        actualModuleState[3] = new SwerveModuleState(bR.getDriveVelocityMeterPerSec(), bR.getAngleRotation2d());

    }
    

    public Rotation2d getRotation(){
        //report pose rotation instead of from gyro because it accounts for heading resets
        return m_pose.getRotation();
    }

    public void resetHeading(){
        //sets the forward direction for field oriented, pose estimator handles all the offsets since it's where we get our rotation from
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
            m_poseEstimator.resetRotation(Rotation2d.k180deg);
        }
        else{
            m_poseEstimator.resetRotation(Rotation2d.kZero);
        }
    }
}
