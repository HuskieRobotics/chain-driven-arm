package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.chainArm.ChainDrivenArmIOInputsAutoLogged;

/** Generic subsystem hardware interface. */
public interface ChainDrivenArmIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  // FIXME: re-evaluate based on what units we want (degreees vs. radians)
  // FIXME: delete properties that aren't supported by Rev
  public static class ChainDrivenArmIOInputs {
    double positionDeg = 0.0;
    double velocityRPM = 0.0;
    double closedLoopError = 0.0;
    double setpoint = 0.0;
    double power = 0.0;
    String controlMode = "";
    double tempCelsius = 0.0;
    double supplyCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ChainDrivenArmIOInputsAutoLogged inputs) {}

  /**
   * Set the motor power to the specified percentage of maximum power.
   *
   * @param power the percentage of maximum power to set the motor to
   */
  public default void setMotorPower(double power) {}

  /**
   * Set the arm position in degrees above the horizontal
   *
   * @param position the position to set the motor to in degrees
   */
  public default void setMotorPosition(double position) {}

  public default double getPosition() {return 0;}
  
  public default boolean atPosition() {return false;}
}
