// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class CANBusIDs{
    
  }

  public static class DrivetrainConstants{

    public static final double radius = .3906711;

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = radius * Math.sqrt(2);
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.2794;

    

    public static final int DRIVETRAIN_PIGEON_ID = 10;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 15;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 16;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 16;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(90.87 + 180.0);
    

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 3;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(35.59 + 180.0);
    

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 18;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 17;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 17;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(110.21 + 180.0);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 2;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 1;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 1;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(78.13 + 180.0);

    public static final double kDrivetrainTrackwidthMeters = 1;
    public static final double kDrivetrainWheelbaseMeters = 1;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
