// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DriveTrain;


public class RobotContainer {

  private final  CommandXboxController m_controller = new CommandXboxController(0);
  private static DriveTrain m_drivetrainSubsystem = new DriveTrain();

  public SendableChooser<Command> m_auto_chooser;

  public static Command testPath;

  public RobotContainer() {

    DefaultDriveCommand ddc = new DefaultDriveCommand(m_drivetrainSubsystem,
      () -> -modifyAxis(m_controller.getLeftY()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(m_controller.getLeftX()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(m_controller.getRightX()) * DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND );
    m_drivetrainSubsystem.setDefaultCommand(ddc);
    
    configureBindings();
    

    m_auto_chooser = new SendableChooser<>();


    m_auto_chooser.setDefaultOption("TestPath1", testPath);

    SmartDashboard.putData(m_auto_chooser);
  }

  public DriveTrain getDriveTrain(){
    return m_drivetrainSubsystem;
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
