// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Escalator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EscalatorMove extends Command {
  Escalator es;
  CommandXboxController x;
  /** Creates a new EscalatorMove. */
  public EscalatorMove(Escalator es, CommandXboxController x) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.es = es;
    this.x = x;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (x.getRightTriggerAxis() > 0.1) {
      es.move(x.getRightTriggerAxis());
    }
    if (x.getLeftTriggerAxis() > 0.1) {
      es.move(-x.getLeftTriggerAxis());
    }
    if (x.getLeftTriggerAxis() < 0.1 && x.getRightTriggerAxis() < 0.1) {
      es.move(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
