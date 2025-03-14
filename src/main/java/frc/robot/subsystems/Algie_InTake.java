// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Algie_InTake extends SubsystemBase {
  /** Creates a new InTake. */
  SparkMax intake;
  SparkMax UpDownMotor;
  double basePosition;
  ShuffleboardTab tab = Shuffleboard.getTab("Algie Intake");
  PIDController pid = new PIDController(0.2, 0, 0);
  private double setPoint = 0.0;
  public double getSetPoint() {
    return setPoint;
  }
  public void setSetPoint(double setPoint) {
    this.setPoint = setPoint;
  }
  public double getBasePosition() {
    return basePosition;
  }
  public void setBasePosition(double basePosition) {
    this.basePosition = basePosition;
  }
  public Algie_InTake() {
    //intake = new SparkMax(RobotMap.moduleintake, MotorType.kBrushless);
    UpDownMotor = new SparkMax(RobotMap.algieintake,MotorType.kBrushless);
    intake = new SparkMax(RobotMap.algieintaketop,MotorType.kBrushless);
    
    tab.addNumber("Algie Encoder ", ()->getPosition());
    tab.addNumber("Algie Setpooint ", ()->getSetPoint());
    
    pid.setSetpoint(0);

    //setBasePosition(getPosition());
    
    pid.setTolerance(0.1);
    
  }
  public void moveIntake(double s){
    intake.set(s);
  }

  public double getPosition(){
    return UpDownMotor.getEncoder().getPosition();
  }
  public void moveUpDown(double s){
    UpDownMotor.set(s);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double s = pid.calculate(getPosition(), setPoint);
    moveUpDown(s);
  }
}
