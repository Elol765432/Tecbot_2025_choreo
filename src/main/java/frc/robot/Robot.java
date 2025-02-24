// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.Optional;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveTrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
 

  private final DriveTrain driveTrain;
  public static AutoFactory autoFactory;

  private static RobotContainer m_robotContainer;
  
  SendableChooser<Command> m_chooser;
  
    private final Timer timer;

    //private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("TestPath1");
    public static Command testPath;
      
  public Robot() {
          m_robotContainer = new RobotContainer();
          driveTrain = Robot.getRobotContainer().getDriveTrain();
      
          autoFactory = new AutoFactory(
           driveTrain::getPose2d,
           driveTrain::resetPose,
           driveTrain::followTrajectory,
           true, 
           driveTrain);
      
          
          timer = new Timer();
    
          testPath = autoFactory.trajectoryCmd("TestPath1");
    }
    
    public static RobotContainer getRobotContainer(){
      return m_robotContainer;
  }

    /*public static AutoFactory getAutoFactory(){
      return autoFactory;
    }*/

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

  private boolean isRedAlliance() {
      return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }

  @Override
  public void autonomousInit() {
            /*if (trajectory.isPresent()) {
            Optional<Pose2d> initialPose = trajectory.get().getInitialPose(isRedAlliance());

            if (initialPose.isPresent()) {
                driveTrain.resetPose(initialPose.get());
            }
        }*/

        timer.restart();

    testPath.schedule();
        
    
  }

  @Override
  public void autonomousPeriodic() {
    /*
       if (trajectory.isPresent()) {
      Optional<SwerveSample> sample = trajectory.get().sampleAt(timer.get(), isRedAlliance());

      if (sample.isPresent()) {
          driveTrain.followTrajectory(sample);
      }
          
  }*/
  }

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
