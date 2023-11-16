package frc.robot.subsystems.chainArm;

import org.littletonrobotics.junction.AutoLog;

/** Generic subsystem hardware interface. */
public interface ChainDrivenArmIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ChainDrivenArmIOInputs {
    double positionRad = 0.0;
    double velocityRPM = 0.0;
    double closedLoopError = 0.0;
    double setpoint = 0.0;
    double power = 0.0;
    String controlMode = "";
    double statorCurrentAmps = 0.0;
    double tempCelsius = 0.0;
    double supplyCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ChainDrivenArmIOInputs inputs) {}

  // FOR TYLER:
  // IMPLEMENT THIS METHOD
  // JUST FIND THE CanSparkMax method to set power and in teh ChainDrivenArmIOTalonFX.java file fill
  // out this method
  // ACTUALLY FIRST MAKE SURE YOU CHANGE THE MOTOR TYPES TO CanSparkMax IN THE
  // ChainDrivenArmIOTalonFX.java FILE
  // AND GET RID OF THE ERRORS

  // ADDITIONALLY TAKE THIS ONE FILE AT A TIME
  // START IN THIS FILE AND THEN PROCEED IN THIS ORDER:
  // ChainDrivenArmIO.java
  // ChainDrivenArmIOTalonFX.java
  // ChainDrivenArm.java

  // FILL OUT THE CONSTANTS CLASS AS NEEDED

  /**
   * Set the motor power to the specified percentage of maximum power.
   *
   * @param power the percentage of maximum power to set the motor to
   */
  public default void setMotorPower(double power) {}

  public default void setMotorCurrent(double current) {}

  /**
   * Set the arm position in degrees above the horizontal
   *
   * @param position the position to set the motor to in degrees
   * @param arbitraryFeedForward the arbitrary feed forward as a percentage of maximum power
   */
  public default void setMotorPosition(double position) {}
}
