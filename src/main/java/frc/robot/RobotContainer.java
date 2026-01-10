// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.swerve.Gyro;
import frc.robot.swerve.Swervedrive;
import frc.robot.commands.DriveCommands;

public class RobotContainer {

    private Swervedrive swerve;
    CommandXboxController driveController = new CommandXboxController(Constants.ControlConstants.k_driverPort);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(
            DriveCommands.joystickDriveFieldOriented(swerve, driveController::getLeftX, driveController::getLeftY, driveController::getRightX) 
                );

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
