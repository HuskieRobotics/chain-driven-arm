package frc.robot.configs;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve.SwerveModuleConstants.SwerveType;

/*
 * Refer to the README for how to represent your robot's configuration. For more information on
 * these methods, refer to the documentation in the RobotConfig class.
 */
public class NovaRobotConfig extends RobotConfig {

  private static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 13;
  private static final int FRONT_LEFT_MODULE_STEER_MOTOR = 12;
  private static final int FRONT_LEFT_MODULE_STEER_ENCODER = 14;
  private static final double FRONT_LEFT_MODULE_STEER_OFFSET_ROT = -0.104492;

  private static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 16;
  private static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 15;
  private static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 17;
  private static final double FRONT_RIGHT_MODULE_STEER_OFFSET_ROT = -0.277832;

  private static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
  private static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
  private static final int BACK_LEFT_MODULE_STEER_ENCODER = 8;
  private static final double BACK_LEFT_MODULE_STEER_OFFSET_ROT = -0.749512;

  private static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 10;
  private static final int BACK_RIGHT_MODULE_STEER_MOTOR = 9;
  private static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11;
  private static final double BACK_RIGHT_MODULE_STEER_OFFSET_ROT = -0.024414;

  private static final int GYRO_ID = 3;

  private static final double TRACKWIDTH_METERS = 0.523875; // 20.625
  private static final double WHEELBASE_METERS = 0.52705; // 20.75
  private static final double ROBOT_WIDTH_WITH_BUMPERS = 0.8382; // meters //33 in
  private static final double ROBOT_LENGTH_WITH_BUMPERS = 0.8382; // meters // 33 in

  /* Angle Motor PID Values */
  private static final double ANGLE_KP = 100.0;
  private static final double ANGLE_KI = 0.0;
  private static final double ANGLE_KD = 0.0;

  private static final double ANGLE_KS = 0.17853;
  private static final double ANGLE_KV = 0.43784;
  private static final double ANGLE_KA = 0.0;

  /* Drive Motor PID Values */
  private static final double DRIVE_KP = 0.2402346041055719;
  private static final double DRIVE_KI = 0.0;
  private static final double DRIVE_KD = 0.013212903225806451;

  private static final double DRIVE_KS = 0.37182;
  private static final double DRIVE_KV = 2.72624;
  private static final double DRIVE_KA = 0.0;

  private static final SwerveType SWERVE_TYPE = SwerveType.MK4I;

  private static final double MAX_VELOCITY_METERS_PER_SECOND = 4.78;
  private static final double MAX_COAST_VELOCITY_METERS_PER_SECOND = 0.05;
  private static final double SLOW_MODE_MULTIPLIER = 0.75;

  private static final double MAX_DRIVE_ACCELERATION_METERS_PER_SECOND_SQUARED = 10.0;
  private static final double MAX_TURN_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 4.0 * Math.PI;

  private static final String CAN_BUS_NAME = "canbus1";

  private static final String CAMERA_NAME_0 = "OV9281";

  private static final String CAMERA_NAME_1 = "OV9281R";

  private static final Transform3d ROBOT_TO_CAMERA_0 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-10.406),
              Units.inchesToMeters(6.603),
              Units.inchesToMeters(49.240)),
          new Rotation3d(0, Units.degreesToRadians(25), Units.degreesToRadians(30)));

  private static final Transform3d ROBOT_TO_CAMERA_1 =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-10.406),
              Units.inchesToMeters(-6.603),
              Units.inchesToMeters(49.240)),
          new Rotation3d(0, Units.degreesToRadians(25), Units.degreesToRadians(-30)));

  private static final double AUTO_MAX_SPEED_METERS_PER_SECOND = 2.0;
  private static final double AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.0;
  private static final double AUTO_DRIVE_P_CONTROLLER = 8.0;
  private static final double AUTO_DRIVE_I_CONTROLLER = 0.0;
  private static final double AUTO_DRIVE_D_CONTROLLER = 0.0;
  private static final double AUTO_TURN_P_CONTROLLER = 0.2;
  private static final double AUTO_TURN_I_CONTROLLER = 0.0;
  private static final double AUTO_TURN_D_CONTROLLER = 0.0;

  // Drive to Pose constants
  private static final double DRIVE_TO_POSE_DRIVE_KP = 2.5;
  private static final double DRIVE_TO_POSE_DRIVE_KD = 0.0;
  private static final double DRIVE_TO_POSE_THETA_KP = 18.0;
  private static final double DRIVE_TO_POSE_THETA_KI = 10.0;
  private static final double DRIVE_TO_POSE_THETA_KD = 0.0;
  private static final double DRIVE_TO_POSE_DRIVE_TOLERANCE_METERS = 0.08;
  private static final double DRIVE_TO_POSE_THETA_TOLERANCE_RADIANS = 0.008;

  private static final double SQUARING_SPEED_METERS_PER_SECOND = 1.0;

  @Override
  public boolean getPhoenix6Licensed() {
    return true;
  }

  @Override
  public double getSwerveAngleKP() {
    return ANGLE_KP;
  }

  @Override
  public double getSwerveAngleKI() {
    return ANGLE_KI;
  }

  @Override
  public double getSwerveAngleKD() {
    return ANGLE_KD;
  }

  @Override
  public double getSwerveAngleKS() {
    return ANGLE_KS;
  }

  @Override
  public double getSwerveAngleKV() {
    return ANGLE_KV;
  }

  @Override
  public double getSwerveAngleKA() {
    return ANGLE_KA;
  }

  @Override
  public double getSwerveDriveKP() {
    return DRIVE_KP;
  }

  @Override
  public double getSwerveDriveKI() {
    return DRIVE_KI;
  }

  @Override
  public double getSwerveDriveKD() {
    return DRIVE_KD;
  }

  @Override
  public double getDriveKS() {
    return DRIVE_KS;
  }

  @Override
  public double getDriveKV() {
    return DRIVE_KV;
  }

  @Override
  public double getDriveKA() {
    return DRIVE_KA;
  }

  @Override
  public SwerveType getSwerveType() {
    return SWERVE_TYPE;
  }

  @Override
  public int[] getSwerveDriveMotorCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_DRIVE_MOTOR,
      FRONT_RIGHT_MODULE_DRIVE_MOTOR,
      BACK_LEFT_MODULE_DRIVE_MOTOR,
      BACK_RIGHT_MODULE_DRIVE_MOTOR
    };
  }

  @Override
  public int[] getSwerveSteerMotorCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_STEER_MOTOR,
      FRONT_RIGHT_MODULE_STEER_MOTOR,
      BACK_LEFT_MODULE_STEER_MOTOR,
      BACK_RIGHT_MODULE_STEER_MOTOR
    };
  }

  @Override
  public int[] getSwerveSteerEncoderCANIDs() {
    return new int[] {
      FRONT_LEFT_MODULE_STEER_ENCODER,
      FRONT_RIGHT_MODULE_STEER_ENCODER,
      BACK_LEFT_MODULE_STEER_ENCODER,
      BACK_RIGHT_MODULE_STEER_ENCODER
    };
  }

  @Override
  public double[] getSwerveSteerOffsets() {
    return new double[] {
      FRONT_LEFT_MODULE_STEER_OFFSET_ROT,
      FRONT_RIGHT_MODULE_STEER_OFFSET_ROT,
      BACK_LEFT_MODULE_STEER_OFFSET_ROT,
      BACK_RIGHT_MODULE_STEER_OFFSET_ROT
    };
  }

  @Override
  public int getGyroCANID() {
    return GYRO_ID;
  }

  @Override
  public double getTrackwidth() {
    return TRACKWIDTH_METERS;
  }

  @Override
  public double getWheelbase() {
    return WHEELBASE_METERS;
  }

  @Override
  public double getRobotWidthWithBumpers() {
    return ROBOT_WIDTH_WITH_BUMPERS;
  }

  @Override
  public double getRobotLengthWithBumpers() {
    return ROBOT_LENGTH_WITH_BUMPERS;
  }

  @Override
  public Transform3d[] getRobotToCameraTransforms() {
    return new Transform3d[] {ROBOT_TO_CAMERA_0, ROBOT_TO_CAMERA_1};
  }

  @Override
  public double getRobotMaxVelocity() {
    return MAX_VELOCITY_METERS_PER_SECOND;
  }

  @Override
  public double getRobotMaxDriveAcceleration() {
    return MAX_DRIVE_ACCELERATION_METERS_PER_SECOND_SQUARED;
  }

  @Override
  public double getRobotMaxTurnAcceleration() {
    return MAX_TURN_ACCELERATION_RADIANS_PER_SECOND_SQUARED;
  }

  @Override
  public double getRobotSlowModeMultiplier() {
    return SLOW_MODE_MULTIPLIER;
  }

  @Override
  public double getRobotMaxCoastVelocity() {
    return MAX_COAST_VELOCITY_METERS_PER_SECOND;
  }

  @Override
  public double getAutoMaxSpeed() {
    return AUTO_MAX_SPEED_METERS_PER_SECOND;
  }

  @Override
  public double getAutoMaxAcceleration() {
    return AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;
  }

  @Override
  public double getAutoDriveKP() {
    return AUTO_DRIVE_P_CONTROLLER;
  }

  @Override
  public double getAutoDriveKI() {
    return AUTO_DRIVE_I_CONTROLLER;
  }

  @Override
  public double getAutoDriveKD() {
    return AUTO_DRIVE_D_CONTROLLER;
  }

  @Override
  public double getAutoTurnKP() {
    return AUTO_TURN_P_CONTROLLER;
  }

  @Override
  public double getAutoTurnKI() {
    return AUTO_TURN_I_CONTROLLER;
  }

  @Override
  public double getAutoTurnKD() {
    return AUTO_TURN_D_CONTROLLER;
  }

  @Override
  public String getCANBusName() {
    return CAN_BUS_NAME;
  }

  @Override
  public String[] getCameraNames() {
    return new String[] {CAMERA_NAME_0, CAMERA_NAME_1};
  }

  @Override
  public double getDriveToPoseDriveKP() {
    return DRIVE_TO_POSE_DRIVE_KP;
  }

  @Override
  public double getDriveToPoseDriveKD() {
    return DRIVE_TO_POSE_DRIVE_KD;
  }

  @Override
  public double getDriveToPoseThetaKI() {
    return DRIVE_TO_POSE_THETA_KI;
  }

  @Override
  public double getDriveToPoseThetaKP() {
    return DRIVE_TO_POSE_THETA_KP;
  }

  @Override
  public double getDriveToPoseThetaKD() {
    return DRIVE_TO_POSE_THETA_KD;
  }

  @Override
  public double getDriveToPoseDriveMaxVelocity() {
    return 0.5;
  }

  @Override
  public double getDriveToPoseDriveMaxAcceleration() {
    return getAutoMaxAcceleration();
  }

  @Override
  public double getDriveToPoseTurnMaxVelocity() {
    return getDriveToPoseDriveMaxVelocity()
        / Math.hypot(getTrackwidth() / 2.0, getWheelbase() / 2.0);
  }

  @Override
  public double getDriveToPoseTurnMaxAcceleration() {
    return getDriveToPoseDriveMaxAcceleration()
        / Math.hypot(getTrackwidth() / 2.0, getWheelbase() / 2.0);
  }

  @Override
  public double getDriveToPoseDriveTolerance() {
    return DRIVE_TO_POSE_DRIVE_TOLERANCE_METERS;
  }

  @Override
  public double getDriveToPoseThetaTolerance() {
    return DRIVE_TO_POSE_THETA_TOLERANCE_RADIANS;
  }

  @Override
  public int getPneumaticsHubCANID() {
    return 0;
  }

  @Override
  public double getMoveToPathFinalVelocity() {
    return SQUARING_SPEED_METERS_PER_SECOND;
  }
}
