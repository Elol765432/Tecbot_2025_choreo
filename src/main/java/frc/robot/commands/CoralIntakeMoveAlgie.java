// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.Coral_InTake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntakeMoveAlgie extends Command {
  /** Creates a new Coral_Intake_Move. */
  Coral_InTake ci;
  ParallelRaceGroup out_p;

  CoralIntakeMoveAlgieOut out;
  double s;
  public CoralIntakeMoveAlgie(Coral_InTake ci, double s) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ci = ci;
    this.s = s;
    addRequirements(ci);

    out_p = new CoralIntakeMoveAlgieOut(ci, s).withTimeout(0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ci.move(s);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    out_p.schedule();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
