package com.swervedrivespecialties.swervelib.rev;


import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.swervedrivespecialties.swervelib.DriveController;
import com.swervedrivespecialties.swervelib.DriveControllerFactory;
import com.swervedrivespecialties.swervelib.MechanicalConfiguration;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import static com.swervedrivespecialties.swervelib.rev.RevUtils.checkNeoError;

public final class NeoDriveControllerFactoryBuilder {
    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    public NeoDriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public NeoDriveControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer id, String _canbus, MechanicalConfiguration mechConfiguration) {
            
            SparkMax motor = new SparkMax(id , MotorType.kBrushless);
            SparkMaxConfig motorConfig = new SparkMaxConfig();
            motorConfig.idleMode(IdleMode.kBrake);
            motorConfig.inverted(mechConfiguration.isDriveInverted());
            // Setup voltage compensation
            if (hasVoltageCompensation()) {
                motorConfig.voltageCompensation(nominalVoltage);
                //dont know how to replicate this: checkNeoError(motor.enableVoltageCompensation(nominalVoltage), "Failed to enable voltage compensation");
            }

            if (hasCurrentLimit()) {
                motorConfig.smartCurrentLimit((int)currentLimit);
                //dont know how to replicate this: c checkNeoError(motor.setSmartCurrentLimit((int) currentLimit), "Failed to set current limit for NEO");
            } 

            
            // !!! NOT NEEDED? apparently there is no way to set this
            //checkNeoError(motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus0, 100), "Failed to set periodic status frame 0 rate");
            //checkNeoError(motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus1, 20), "Failed to set periodic status frame 1 rate");
            //checkNeoError(motor.setPeriodicFramePeriod(SparkLowLevel.PeriodicFrame.kStatus2, 20), "Failed to set periodic status frame 2 rate");
            
            // Set neutral mode to brake
            // alñready done in the new way, below constructor
            //motor.setIdleMode(SparkMax.IdleMode .IdleMode.kBrake);

            // Setup encoder
            RelativeEncoder encoder = motor.getEncoder();
            double positionConversionFactor = Math.PI * mechConfiguration.getWheelDiameter() * mechConfiguration.getDriveReduction();
            motorConfig.encoder.positionConversionFactor(positionConversionFactor).velocityConversionFactor(positionConversionFactor);

            // previous way
            //encoder.setPositionConversionFactor(positionConversionFactor);
            //encoder.setVelocityConversionFactor(positionConversionFactor / 60.0);
            motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

            return new ControllerImplementation(motor, encoder);
        }
    }

    private static class ControllerImplementation implements DriveController {
        private final SparkMax motor;
        private final RelativeEncoder encoder;

        private ControllerImplementation(SparkMax motor, RelativeEncoder encoder) {
            this.motor = motor;
            this.encoder = encoder;
        }

        @Override
        public MotorController getDriveMotor() {
            return this.motor;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            motor.setVoltage(voltage);
        }

        @Override
        public double getStateVelocity() {
            return encoder.getVelocity();
        }

        @Override
        public double getStateDistance() {
            return encoder.getPosition();
        }
    }
}
