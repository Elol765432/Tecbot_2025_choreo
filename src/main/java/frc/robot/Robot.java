// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.Coral_Intake_Move;
import frc.robot.commands.Elevator_Levels;
import frc.robot.commands.ResetPose;
import frc.robot.subsystems.Coral_InTake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  
  public static DriveTrain driveTrain;
  public static Elevator elevator;
  public static AutoFactory autoFactory;

  private final RobotContainer m_robotContainer;

  private final AutoChooser autoChooser;

  public Robot() {
    m_robotContainer = new RobotContainer();
    driveTrain = m_robotContainer.getDriveTrain();
    elevator = new Elevator();

              autoFactory = new AutoFactory(
           driveTrain::getPose2d,
           driveTrain::resetPose,
           driveTrain::followTrajectory,
           true, 
           driveTrain);

    autoChooser = new AutoChooser();
    
    autoChooser.addRoutine("Example Routine", this::exampleRoutine);

    SmartDashboard.putData(autoChooser);

    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
  }

  private AutoRoutine exampleRoutine(){
    AutoRoutine routine = autoFactory.newRoutine("exampleRoutine");

    AutoTrajectory startToS1 = routine.trajectory("StartToS1");
    AutoTrajectory s1ToS2 = routine.trajectory("S1ToS2");
    AutoTrajectory s2ToS3 = routine.trajectory("S2ToS3");

    routine.active().onTrue(Commands.sequence(new ResetPose(driveTrain),startToS1.cmd()));

    startToS1.done().onTrue(new SequentialCommandGroup(
      new Elevator_Levels(m_robotContainer.getElevator(), 4, 0, 0), 
      new Coral_Intake_Move(m_robotContainer.getCoral_InTake(),.2),
      new Coral_Intake_Move(m_robotContainer.getCoral_InTake(), -RobotMap.coralspeed)).andThen(new SequentialCommandGroup(new Elevator_Levels(elevator, 0, 0, 0), s1ToS2.cmd())));

    s1ToS2.done().onTrue(new Coral_Intake_Move(m_robotContainer.getCoral_InTake(), RobotMap.coralspeed).andThen(s2ToS3.cmd()));

    s2ToS3.done().onTrue(new SequentialCommandGroup(
      new Elevator_Levels(m_robotContainer.getElevator(), 4, 0, 0), 
      new Coral_Intake_Move(m_robotContainer.getCoral_InTake(),.2),
      new Coral_Intake_Move(m_robotContainer.getCoral_InTake(), -RobotMap.coralspeed)));

    return routine;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();


  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
