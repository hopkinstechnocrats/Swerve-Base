package frc.robot.swerve;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase{
    
    Pigeon2 pigeon;

    Gyro(int gyroID){
        pigeon = new Pigeon2(gyroID, new CANBus("GertrudeGreyser"));
    }

    public Rotation2d getRotation(){
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
            return pigeon.getRotation2d().plus(Rotation2d.k180deg); 
        }else{
            return pigeon.getRotation2d();
        }
    }

    public double getAccelX(){
        return pigeon.getAccelerationX().getValueAsDouble();
    }

    public double getAccelY(){
        return pigeon.getAccelerationY().getValueAsDouble();
    }

    public double getAccelZ(){
        return pigeon.getAccelerationZ().getValueAsDouble();
    }

}
