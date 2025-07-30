package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.swerve.Swervedrive;

public class DriveCommands{
    private DriveCommands(){}
  
    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {

        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(
            Math.hypot(x, y), Constants.ControlConstants.k_driveControllerDeadband);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    }

    public static Command joystickDriveFieldOriented(Swervedrive swervedrive,
            double xVelocity, double yVelocity, double omega){
        return Commands.run(() -> {}, swervedrive);
    }
    
}
