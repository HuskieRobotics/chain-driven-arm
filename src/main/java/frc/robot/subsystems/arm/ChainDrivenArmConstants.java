package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;

public class ChainDrivenArmConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ChainDrivenArmConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean DEBUGGING = false;
  public static final boolean TESTING = false;
  public static final String SUBSYSTEM_NAME = "Chain Driven Arm";

  public static final int MOTOR_ONE_ID = 20; //FIXME: set the motor IDs
  public static final int MOTOR_TWO_ID = 0;
  public static final int MOTOR_THREE_ID = 0;
  public static final int MOTOR_FOUR_ID = 0;

  public static final double GEAR_RATIO = 100.0;
  public static final boolean MOTOR_INVERTED = false;

  public static final int COUNTS_PER_REV = 8192; //FIXME: took this # from a SPARK_MAX example
  public static final CANSparkMax.IdleMode MODE = CANSparkMax.IdleMode.kCoast;

  public static final double POSITION_PID_P = 0.0;
  public static final double POSITION_PID_I = 0;
  public static final double POSITION_PID_D = 0;
  public static final double POSITION_PID_PEAK_OUTPUT = 0.01;
  public static final double POSITION_FEEDFORWARD = 0;

  public static final int CONTINUOUS_CURRENT_LIMIT = 40;
  public static final double PEAK_CURRENT_LIMIT = 50;
  public static final double PEAK_CURRENT_DURATION = 0.5;

  public static final int TIMEOUT_MS = 30;
  public static final int SLOT_INDEX = 0;
}
