// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  public static final double[] SETPOINTS = new double[]{0.0, 6.96, 14.32, 25.41 , 41.00};
  SparkMax elevatorMotor1;
  SparkMax elevatorMotro2;
  Tej_Subs lights;
  ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
  double basePosition;
  int level = 0;
  public int getLevel() {
    int lev = 0;
    double minSubs = 10000;
    for(int  i= 0; i < SETPOINTS.length; i++){

      double subs = Math.abs(SETPOINTS[i] - getPosition());
      if(subs < minSubs){
          minSubs = subs;
          lev = i;
      }

    }
    return lev;
  }
  public void setLevel(int level) {
    this.level = level;
  }
  public double getBasePosition() {
    return basePosition;
  }
  public void setBasePosition(double basePosition) {
    this.basePosition = basePosition;
  }
  public Elevator() {
    elevatorMotor1 = new SparkMax(RobotMap.elevatorMotor1, MotorType.kBrushless);
    elevatorMotro2 = new SparkMax(RobotMap.elevatorMotor2, MotorType.kBrushless);
    this.setBasePosition(getPosition());
    lights = Tej_Subs.getInstance();
    tab.addNumber("Encoder Value", ()-> getPosition() );
    tab.addNumber("Absolute Encoder", ()-> elevatorMotor1.getAbsoluteEncoder().getPosition() );
  }
  public void elevatorMove(double s){
    elevatorMotor1.set(s);
    elevatorMotro2.set(-s);
  }
  public void setLightMode(int mode){
    lights.setMode(mode);
  }   
  public double getPosition(){
    //System.out.println(elevatorMotor1.getEncoder().getPosition());
    return elevatorMotor1.getEncoder().getPosition();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
