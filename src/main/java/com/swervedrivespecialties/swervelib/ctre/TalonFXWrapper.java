// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.swervedrivespecialties.swervelib.ctre;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/** This class implements the things MotorController needs, cannot be done as an extension to TalonFX as there are method conflicts.  */
public class TalonFXWrapper implements MotorController{

    private TalonFX motor;

    public TalonFXWrapper(){
    }

    public TalonFXWrapper(TalonFX motor){
        this.motor = motor;
    }    

    @Override
    public double get() {
        // TODO Auto-generated method stub
        return motor.get();
    }

    @Override
    public void set(double speed) {
        // TODO Auto-generated method stub
        motor.set(speed);
    }

    
    public void setInverted(boolean isInverted) {
        // TODO Auto-generated method stub
        motor.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        // TODO Auto-generated method stub
        return motor.getInverted();
    }

    @Override
    public void stopMotor() {
        // TODO Auto-generated method stub
        motor.stopMotor();
    }

    @Override
    public void disable() {
        // TODO Auto-generated method stub
        motor.disable();
    }
    
}
