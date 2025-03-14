// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Coral_InTake extends SubsystemBase {
  /** Creates a new Coral_InTake. */
  SparkMax m1;
  SparkMax m2;
  public Coral_InTake() {
    m1 = new SparkMax(RobotMap.coralIntake1, MotorType.kBrushless);
    m2 = new SparkMax(RobotMap.coralIntake2, MotorType.kBrushless);
  }
  public void move(double s){
    m1.set(s);
    m2.set(s);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
