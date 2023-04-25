// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.CANBusIDs;
import frc.robot.Constants.DrivetrainConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Drivetrain extends SubsystemBase {

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private final Pigeon2 gyro = new Pigeon2(CANBusIDs.pigeonID);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    // Front left
    new Translation2d(DrivetrainConstants.kDrivetrainTrackwidthMeters / 2.0, DrivetrainConstants.kDrivetrainWheelbaseMeters / 2.0),
    // Front right
    new Translation2d(DrivetrainConstants.kDrivetrainTrackwidthMeters / 2.0, -DrivetrainConstants.kDrivetrainWheelbaseMeters / 2.0),
    // Back left
    new Translation2d(-DrivetrainConstants.kDrivetrainTrackwidthMeters / 2.0, DrivetrainConstants.kDrivetrainWheelbaseMeters / 2.0),
    // Back right
    new Translation2d(-DrivetrainConstants.kDrivetrainTrackwidthMeters / 2.0, -DrivetrainConstants.kDrivetrainWheelbaseMeters / 2.0));

    // private final SwerveDrivePoseEstimator estimator;

    

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    Mk4ModuleConfiguration mk4ModuleConfiguration = new Mk4ModuleConfiguration();
    mk4ModuleConfiguration.setDriveCurrentLimit(50);
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    // make swerve modules and set up shuffleboard for them
    frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
            mk4ModuleConfiguration, Mk4iSwerveModuleHelper.GearRatio.L3, DrivetrainConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            DrivetrainConstants.FRONT_LEFT_MODULE_STEER_MOTOR, DrivetrainConstants.FRONT_LEFT_MODULE_STEER_ENCODER, DrivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET);
    frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
            mk4ModuleConfiguration, Mk4iSwerveModuleHelper.GearRatio.L3, DrivetrainConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_MOTOR, DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_ENCODER, DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_OFFSET);
    backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
            mk4ModuleConfiguration, Mk4iSwerveModuleHelper.GearRatio.L3, DrivetrainConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
            DrivetrainConstants.BACK_LEFT_MODULE_STEER_MOTOR, DrivetrainConstants.BACK_LEFT_MODULE_STEER_ENCODER, DrivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET);
    backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
            mk4ModuleConfiguration, Mk4iSwerveModuleHelper.GearRatio.L3, DrivetrainConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            DrivetrainConstants.BACK_RIGHT_MODULE_STEER_MOTOR, DrivetrainConstants.BACK_RIGHT_MODULE_STEER_ENCODER, DrivetrainConstants.BACK_RIGHT_MODULE_STEER_OFFSET);

    
    
    
  }

  //--------------- System State ---------------------

  public double getYaw(){
    return gyro.getYaw();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
