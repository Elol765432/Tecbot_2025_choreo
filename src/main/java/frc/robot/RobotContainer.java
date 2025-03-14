// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AlgieTake;
import frc.robot.commands.AlgieUpDownMotorControl;
import frc.robot.commands.CoralIntakeMoveAlgie;
import frc.robot.commands.AlgieDownUpNeumatics;
import frc.robot.commands.Coral_Intake_Move;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.Elevator_Levels;
import frc.robot.commands.Elevator_Move;
import frc.robot.commands.EscalatorMove;
import frc.robot.subsystems.Algie_InTake;
import frc.robot.subsystems.Coral_InTake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Escalator;

public class RobotContainer {

  private final  CommandXboxController m_controller = new CommandXboxController(0);
  private final  CommandXboxController m_controller2 = new CommandXboxController(1);
  private final static DriveTrain m_drivetrainSubsystem = new DriveTrain();
   Algie_InTake it = new Algie_InTake();
    static Coral_InTake ci = new Coral_InTake();
      static Elevator elevator = new Elevator();
        static RobotContainer instance;
        private final SendableChooser<Command> m_chooser = new SendableChooser<>();
        ShuffleboardTab tab = Shuffleboard.getTab("Command Selector");
        //Escalator es = new Escalator();
        public static RobotContainer getInstance(){
          return instance;
        }
        public CommandXboxController getPilot(){
          return m_controller;
        }
        public CommandXboxController getCoPilot(){
          return m_controller2;
        }
    
        public RobotContainer() {
          ///////////////////////////////////////////////////////////////////////////////////////////////
          DefaultDriveCommand ddc = new DefaultDriveCommand(m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.getLeftY()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getLeftX()) * DriveTrain.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(m_controller.getRightX()) * DriveTrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND );
          m_drivetrainSubsystem.setDefaultCommand(ddc);
          //////////////////////////////////////////////////////////////////////////////////////////////
          elevator.setDefaultCommand(new Elevator_Move(elevator, m_controller,m_controller2));
          ///////////CONTROL 1/////////////////////////////////////////////////
          m_controller.a().whileTrue(new Coral_Intake_Move(ci,RobotMap.coralspeed));
          m_controller.b().whileTrue(new Coral_Intake_Move(ci,-RobotMap.coralspeed));
         /*m_controller.rightBumper().whileTrue(new AlgieDownUpNeumatics(it, true));
          m_controller.leftBumper().whileTrue(new AlgieDownUpNeumatics(it, false));*/
         // m_controller.a().whileTrue(new AlgieUpDownMotorControl(it, true));
          //m_controller.b().whileTrue(new AlgieUpDownMotorControl(it,false));
         m_controller.y().whileTrue(new AlgieTake(it, RobotMap.algieInputSpeed));
         m_controller.x().whileTrue(new AlgieTake(it, -RobotMap.algieOutputSpeed));
          ///////////CONTROL 2/////////////////////////////////////////////////
          m_controller2.x().whileTrue(new Coral_Intake_Move(ci,-0.1));
         m_controller2.y().whileTrue(new CoralIntakeMoveAlgie(ci, RobotMap.coralspeed));     
         // m_controller2.leftBumper().whileTrue(new AlgieDownUpNeumatics(it, true));
          //m_controller2.rightBumper().whileTrue(new AlgieDownUpNeumatics(it, false));
          //es.setDefaultCommand(new EscalatorMove(es, m_controller2));
          m_controller2.rightBumper().whileTrue(new AlgieTake(it, RobotMap.algieInputSpeed));
          m_controller2.leftBumper().whileTrue(new AlgieTake(it, -RobotMap.algieOutputSpeed));
          m_controller2.a().whileTrue(new Coral_Intake_Move(ci,RobotMap.coralspeed));
          m_controller2.b().whileTrue(new Coral_Intake_Move(ci,-RobotMap.coralspeed));
        m_controller2.button(7).onTrue(new AlgieUpDownMotorControl(0,it));
        m_controller2.button(8).onTrue(new AlgieUpDownMotorControl(1,it));
          m_controller2.povDown().onTrue(new SequentialCommandGroup(new Elevator_Levels(elevator, 1, 0, 0),new Coral_Intake_Move(ci,0.2).withTimeout(0.5)));
          m_controller2.povRight().onTrue(new SequentialCommandGroup(new Elevator_Levels(elevator, 2, 0, 0),new Coral_Intake_Move(ci,0.2).withTimeout(0.5)));
          m_controller2.povUp().onTrue(new SequentialCommandGroup(new Elevator_Levels(elevator, 3, 0, 0),new Coral_Intake_Move(ci,0.2).withTimeout(0.5)));
          m_controller2.povLeft().onTrue(new SequentialCommandGroup(new Elevator_Levels(elevator, 4, 0, 0),new Coral_Intake_Move(ci,0.2).withTimeout(0.5)));
          configureBindings();
          RobotContainer.instance = this;
          tab.add(m_chooser);
        }
      
      public static DriveTrain getDriveTrain(){
        return m_drivetrainSubsystem;
    }
    
      public static Elevator getElevator(){
        return elevator;
    }
  
    public static Coral_InTake getCoral_InTake(){
      return ci;
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    
    return m_chooser.getSelected();
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
/*This robot is part of a great family <3:
 * Santiago Ortiz
 * Alex Lopez
 * Alonso Nidea
 * Fer Magaña
 * Fer Sanchez
 * Jesus Mendiola
 * Rodolfo Mendiola
 */