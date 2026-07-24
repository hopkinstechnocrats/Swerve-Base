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
import frc.robot.commands.TeleopDrive;

public class RobotContainer {

    //setup all our objects
    Swervedrive m_swerve = new Swervedrive();
    CommandXboxController driveController = new CommandXboxController(Constants.ControlConstants.k_driverPort);

    public RobotContainer() {
        //default commands will run on subsytems when no other commands are running, all subsystems should have one
        m_swerve.setDefaultCommand(
            new TeleopDrive(m_swerve, () -> driveController.getLeftY(), () -> driveController.getLeftX(), () -> driveController.getRightX(), 
                driveController::getLeftTriggerAxis, driveController::getRightTriggerAxis) 
        );
        

        configureBindings();
    }

    private void configureBindings() {
        //nice place to configure what all the buttons do
        driveController.a().onTrue(Commands.runOnce(() -> m_swerve.resetHeading(), m_swerve)); 
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
