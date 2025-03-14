// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Coral_InTake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Tej_Subs;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Elevator_Levels extends Command {
  /** Creates a new Elevator_Levels_Autonomus. */
  Elevator el;
  int level;
  double setPoint;
  int mode;
  int mode2;
  PIDController pid = new PIDController(1.0, 0  , 0 );
  Timer t;
  private static double[][] posPidTable = new double[][]{ {0.0,0,0} , {0.25,0,0} , {0.25,0,0}, {0.24,0,0}, {0.25,0,0}};
  private static double[][] negPidTable = new double[][]{ {0.0,0,0} , {0.05,0,0} , {0.05,0,0}, {0.05,0,0}, {0.05,0,0}};
  
  public static final int KP = 0;
  public static final double TOLERANCE = 0.1;
  boolean leveIsSameLevel;
    private boolean cancel;
  



    public Elevator_Levels(Elevator el, int level, int mode,int mode2) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.el = el;
      this.level = level;
      this.mode = mode;
      this.mode2 = mode2;
      t = new Timer();
      addRequirements(el);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      int newL = level - el.getLevel();
      leveIsSameLevel = newL == 1;

      if(!leveIsSameLevel){
        double[][] table = newL > 0 ? posPidTable:negPidTable;
        int pos = Math.abs(newL);
        pid.setP( table[pos][KP] );
        pid.setSetpoint(Elevator.SETPOINTS[level]);
        pid.setTolerance(TOLERANCE);
        t.restart();
      }
      cancel = false;
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if (RobotContainer.getInstance().getCoPilot().getRightY()>0.1) {
        cancel = true;
    }
    double s = pid.calculate(el.getPosition());
    el.elevatorMove(s);
    System.out.println("Executing:" +  level +"__}  Position"+ el.getPosition() + " // "  + pid.getP() );
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("FINISHED");
    t.stop();

    //el.setLevel(level);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return pid.atSetpoint()||leveIsSameLevel||cancel||(t.get()>1.8);
  }
}
