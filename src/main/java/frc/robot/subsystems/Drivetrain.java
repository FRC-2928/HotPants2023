package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;

// import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private static final double MAX_VOLTAGE = 12.0;
  /**
   * The maximum velocity of the robot in meters per second.
   * This is a measure of how fast the robot should be able to drive in a straight line.
   * The formula for calculating the theoretical maximum velocity is:
   *  <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * PI
   * The motor free speed for the Falcon500 is 6380
   */
  // TODO Which model do we have? L1, L2, L3, L4
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
    SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
    SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
//     public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.14528;
    
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private final SwerveModulePosition frontLeftPosition = new SwerveModulePosition(0, new Rotation2d(0));
    private final SwerveModulePosition frontRightPosition = new SwerveModulePosition(0, new Rotation2d(0));
    private final SwerveModulePosition backLeftPosition = new SwerveModulePosition(0, new Rotation2d(0));
    private final SwerveModulePosition backRightPosition = new SwerveModulePosition(0, new Rotation2d(0));

    private final Pigeon2 gyroscope = new Pigeon2(DrivetrainConstants.DRIVETRAIN_PIGEON_ID);

    // Locations for the swerve drive modules relative to the robot center.
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-DrivetrainConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DrivetrainConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );
    private final SwerveDriveOdometry odometry; 
      

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // Convert to module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    public Drivetrain() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        // TODO Which model do we have? L1, L2, L3, L4 - L2!
        frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                DrivetrainConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                DrivetrainConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
                DrivetrainConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
                DrivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET
        );

        frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                DrivetrainConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                DrivetrainConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                DrivetrainConstants.BACK_LEFT_MODULE_STEER_MOTOR,
                DrivetrainConstants.BACK_LEFT_MODULE_STEER_ENCODER,
                DrivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET
        );

        backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk4iSwerveModuleHelper.GearRatio.L2,
                DrivetrainConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                DrivetrainConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
                DrivetrainConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
                DrivetrainConstants.BACK_RIGHT_MODULE_STEER_OFFSET
        );

        odometry = new SwerveDriveOdometry(kinematics, 
                              Rotation2d.fromDegrees(gyroscope.getYaw()),
                              new SwerveModulePosition[] {frontLeftPosition,
                                                          frontRightPosition,
                                                          backLeftPosition,
                                                          backRightPosition});
                                
        shuffleboardTab.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
        shuffleboardTab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
        shuffleboardTab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());
    }

    public void zeroGyroscope() {
        odometry.resetPosition(
          Rotation2d.fromDegrees(gyroscope.getYaw()),       
        
                new SwerveModulePosition[] {frontLeftPosition,
                  frontRightPosition,
                  backLeftPosition,
                  backRightPosition},
                  new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)
                )
        );
    }

    public Rotation2d getRotation() {
        return odometry.getPoseMeters().getRotation();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void periodic() {
      
      odometry.update(Rotation2d.fromDegrees(gyroscope.getYaw()),
      new SwerveModulePosition[] {frontLeftPosition,
                                  frontRightPosition,
                                  backLeftPosition,
                                  backRightPosition}
      );

      SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

      frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
      frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
      backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
      backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }
}
