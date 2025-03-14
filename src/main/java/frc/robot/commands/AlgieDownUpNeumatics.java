// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveTranslation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algie_InTake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgieDownUpNeumatics extends Command {
  /** Creates a new DownUp_Intake. */
  Algie_InTake it;
  boolean of;
  public AlgieDownUpNeumatics(Algie_InTake it,boolean of) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.it = it;
    this.of = of;
    addRequirements(it);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (of) {
      //it.goUp();
    }else{
      //it.goDown();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
