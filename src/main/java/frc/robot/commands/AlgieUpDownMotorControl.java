
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Algie_InTake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgieUpDownMotorControl extends Command {
  double setPoint;
  Algie_InTake ai;
  double diff = 3;
  /** Creates a new AlgieUpDownMeoto. */
  public AlgieUpDownMotorControl(double setPoint, Algie_InTake ai) {
    // Use addRequirements() here to declare subsystem dependencies.
   addRequirements(ai);
   this.setPoint = -1*diff*setPoint;
   this.ai = ai;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ai.setSetPoint(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ai.setSetPoint(setPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
