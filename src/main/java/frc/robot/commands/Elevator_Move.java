// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Elevator_Move extends Command {
  /** Creates a new UP_Elevator. */
  Elevator el;
  CommandXboxController x;
  CommandXboxController x2;
  public Elevator_Move(Elevator el,CommandXboxController x, CommandXboxController x2) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.el = el;
    this.x = x;
    this.x2 = x2;
    addRequirements(el);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //////////////////////////////////////////////
   if (x.getRightTriggerAxis()>0) {
    el.elevatorMove(x.getRightTriggerAxis());
   }
   if (x.getLeftTriggerAxis()>0) {
    el.elevatorMove(-x.getLeftTriggerAxis());
   }
   if (Math.abs(x2.getLeftY())>0.08) {
    el.elevatorMove(-x2.getLeftY());
   }
   if (x.getRightTriggerAxis()<0.01 && x.getLeftTriggerAxis()<0.01 &&Math.abs(x2.getLeftY())<0.08 ) {
    el.elevatorMove(0);
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
