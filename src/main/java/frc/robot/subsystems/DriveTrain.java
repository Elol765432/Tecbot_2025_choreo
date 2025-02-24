// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.*;

import java.util.Optional;

public class DriveTrain extends SubsystemBase {

   /*
    * 
    * Three options here:
    * 1. Update the SDS Democat library to support the Kraken, changing the needed configutrations
    * 2. Use the code : https://github.com/FRC4561TerrorBytes/TB2024/ or https://github.com/dirtbikerxz/BaseTalonFXSwerve
    * 3. Use another library as YAGSL ... 
    * 
    * For now, let's test the thing thinking of two neos.... Doesn't appear to be bad.
    */


  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // FIXED!!!  FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */



  public static final double MAX_VELOCITY_METERS_PER_SECOND = 5820.0 / 60.0 *
          SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4I_L2.getWheelDiameter()* Math.PI; // FIXME: this has to changfe for krakens!!
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );



  private final AHRS m_navx = new AHRS(NavXComType.kMXP_SPI); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;
  public static SwerveDriveOdometry odometer;
  private Field2d field;
  
  private SwerveDrivePoseEstimator poseEstimator; 

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private final PIDController xController = new PIDController(1.5, 0.0, 0.0);
  private final PIDController yController = new PIDController(1.5, 0.0, 0.0);
  private final PIDController headingController = new PIDController(3, 0.0, 0.0);

  public DriveTrain() {
    
   ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    // FIXME Setup motor configuration. Here we are believing that ktaken and neos are kindda of the same. 
    
    MotorType driveType = MotorType.FALCON; // the falcon 500 and the kraken are not that different as read on BaseTalonFXSwerve
    MotorType steerType = MotorType.NEO;
  
     m_frontLeftModule = new MkSwerveModuleBuilder()
                .withLayout(tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(driveType, Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR)
                .withSteerMotor(steerType, Constants.FRONT_LEFT_MODULE_STEER_MOTOR)
                .withSteerEncoderPort(Constants.FRONT_LEFT_MODULE_STEER_ENCODER)
                .withSteerOffset(Constants.FRONT_LEFT_MODULE_STEER_OFFSET)
                .build();
     // zero should be facing straight forward on original code.... Do follow instructions on original Readme



    // We will do the same for the other modules
   m_frontRightModule = new MkSwerveModuleBuilder()
                .withLayout(tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(2, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(driveType, Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR)
                .withSteerMotor(steerType, Constants.FRONT_RIGHT_MODULE_STEER_MOTOR)
                .withSteerEncoderPort(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER)
                .withSteerOffset(Constants.FRONT_RIGHT_MODULE_STEER_OFFSET)
                .build();

    m_backLeftModule = new MkSwerveModuleBuilder()
                .withLayout(tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(4, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(driveType, Constants.BACK_LEFT_MODULE_DRIVE_MOTOR)
                .withSteerMotor(steerType, Constants.BACK_LEFT_MODULE_STEER_MOTOR)
                .withSteerEncoderPort(Constants.BACK_LEFT_MODULE_STEER_ENCODER)
                .withSteerOffset(Constants.BACK_LEFT_MODULE_STEER_OFFSET)
                .build();

    m_backRightModule = new MkSwerveModuleBuilder()
                .withLayout(tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                .withSize(2, 4)
                .withPosition(6, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(driveType, Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR)
                .withSteerMotor(steerType, Constants.BACK_RIGHT_MODULE_STEER_MOTOR)
                .withSteerEncoderPort(Constants.BACK_RIGHT_MODULE_STEER_ENCODER)
                .withSteerOffset(Constants.BACK_RIGHT_MODULE_STEER_OFFSET)
                .build();


    odometer = new SwerveDriveOdometry(m_kinematics, getGyroscopeRotation(),
                new SwerveModulePosition[]{ 
                  m_frontLeftModule.getPosition(),
                  m_frontRightModule.getPosition(),
                  m_backLeftModule.getPosition(),
                  m_backRightModule.getPosition()
                }
              );

    poseEstimator = new SwerveDrivePoseEstimator(m_kinematics,
                     getGyroscopeRotation(), 
                     new SwerveModulePosition[]{
                     m_frontLeftModule.getPosition(),
                     m_frontRightModule.getPosition(),
                     m_backLeftModule.getPosition(),
                     m_backRightModule.getPosition(),
                     },  new Pose2d());

    field = new Field2d();
    SmartDashboard.putData("Field", field);                 
                     
    headingController.enableContinuousInput(-Math.PI, Math.PI);   
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
      m_navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
    
    if (m_navx.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }


  public Pose2d getPose2d(){
    return odometer.getPoseMeters();
  }

  public void resetPose(Pose2d pose2d){
    odometer.resetPosition(getGyroscopeRotation(), new SwerveModulePosition[]{ 
                   m_frontLeftModule.getPosition(),
                   m_frontRightModule.getPosition(),
                   m_backLeftModule.getPosition(),
                   m_backRightModule.getPosition()
                 },
                  pose2d);
   }


  @Override
  public void periodic() {

    //System.out.println("Chassis speeds " + m_chassisSpeeds );
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());


    SwerveModulePosition positions[] = new SwerveModulePosition[4];
    positions[0] = m_frontLeftModule.getPosition();
    positions[1] = m_frontRightModule.getPosition();
    positions[2] = m_backLeftModule.getPosition();
    positions[3] = m_backRightModule.getPosition();
    
    odometer.update(getGyroscopeRotation(),positions);
    poseEstimator.update(getGyroscopeRotation(), positions);
    field.setRobotPose(poseEstimator.getEstimatedPosition());

    
    double PositionX = odometer.getPoseMeters().getX();
    double PositionY = odometer.getPoseMeters().getY();
    SmartDashboard.putNumber("X", PositionX);
    SmartDashboard.putNumber("Y", PositionY);

    SmartDashboard.putNumber("Navx", m_navx.getAngle());

  }

   public void followTrajectory(SwerveSample s) {
        Pose2d pose = getPose2d();
        //SwerveSample s = sample.get();
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            s.vx + xController.calculate(pose.getX(), s.x),
            s.vy + yController.calculate(pose.getY(), s.y),
            s.omega + headingController.calculate(pose.getRotation().getRadians(), s.heading), 
            getGyroscopeRotation()
        );
        
        drive(speeds);
   }
}