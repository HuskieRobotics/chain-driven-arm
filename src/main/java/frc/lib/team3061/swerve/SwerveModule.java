/*
 * Initially from https://github.com/Team364/BaseFalconSwerve
 */

package frc.lib.team3061.swerve;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** SwerveModule models a single swerve module. */
public class SwerveModule {
  private final SwerveModuleIO io;
  private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

  private int moduleNumber;
  private double lastAngle;
  private double maxVelocity;
  private double lastAngleMotorVelocity = 0.0;

  private static final String SUBSYSTEM_NAME = "Swerve";
  private static final boolean DEBUGGING = false;

  /**
   * Create a new swerve module.
   *
   * @param io the hardware-abstracted swerve module object
   * @param moduleNumber the module number (0-3)
   * @param maxVelocity the maximum drive velocity of the module in meters per second
   */
  public SwerveModule(SwerveModuleIO io, int moduleNumber, double maxVelocity) {
    this.io = io;
    this.moduleNumber = moduleNumber;
    this.maxVelocity = maxVelocity;

    lastAngle = getState().angle.getDegrees();

    /* set DEBUGGING to true to view values in Shuffleboard. This is useful when determining the steer offset constants. */
    if (DEBUGGING) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.addNumber(
          "Mod " + this.moduleNumber + ": Cancoder", () -> inputs.angleAbsolutePositionDeg);
      tab.addNumber("Mod " + this.moduleNumber + ": Integrated", () -> inputs.anglePositionDeg);
      tab.addNumber(
          "Mod " + this.moduleNumber + ": Velocity", () -> inputs.driveVelocityMetersPerSec);
    }
  }

  /**
   * Set this swerve module to the specified speed and angle.
   *
   * @param desiredState the desired state of the module
   * @param isOpenLoop if true, the drive motor will be set to the calculated fraction of the max
   *     velocity; if false, the drive motor will set to the specified velocity using a closed-loop
   *     controller (PID).
   * @param forceAngle if true, the module will be forced to rotate to the specified angle; if
   *     false, the module will not rotate if the velocity is less than 1% of the max velocity.
   */
  public void setDesiredState(
      SwerveModuleState desiredState, boolean isOpenLoop, boolean forceAngle) {

    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / maxVelocity;
      io.setDriveMotorPercentage(percentOutput);
    } else {
      io.setDriveVelocity(desiredState.speedMetersPerSecond);
    }

    // Unless the angle is forced (e.g., X-stance), don't rotate the module if speed is less then
    // 1%. This prevents jittering if the controller isn't tuned perfectly. Perhaps more
    // importantly, it allows for smooth repeated movement as the wheel direction doesn't reset
    // during pauses (e.g., multi-segmented auto paths).
    double angle;
    if (!forceAngle && Math.abs(desiredState.speedMetersPerSecond) <= (maxVelocity * 0.01)) {
      angle = lastAngle;
    } else {
      angle = desiredState.angle.getDegrees();
    }

    io.setAnglePosition(angle);
    lastAngle = angle;
  }

  /**
   * Set the drive motor to the specified voltage. This is only used for characterization via the
   * FeedForwardCharacterization command. The module will be set to 0 degrees throughout the
   * characterization; as a result, the wheels don't need to be clamped to hold them straight.
   *
   * @param voltage the specified voltage for the drive motor
   */
  public void setVoltageForDriveCharacterization(double voltage) {
    io.setAnglePosition(0.0);
    lastAngle = 0.0;
    io.setDriveMotorPercentage(voltage / 12.0);
  }

  /**
   * Set the angle motor to the specified voltage. This is only used for characterization via the
   * FeedForwardCharacterization command.
   *
   * @param voltage the specified voltage for the angle motor
   */
  public void setVoltageForRotateCharacterization(double voltage) {
    io.setAngleMotorPercentage(voltage / 12.0);
  }

  /**
   * Get the current state of this swerve module.
   *
   * @return the current state of this swerve module
   */
  public SwerveModuleState getState() {
    double velocity = inputs.driveVelocityMetersPerSec;
    Rotation2d angle = Rotation2d.fromDegrees(inputs.anglePositionDeg);
    return new SwerveModuleState(velocity, angle);
  }

  /**
   * Get the current position of this swerve module.
   *
   * @return the current position of this swerve module
   */
  public SwerveModulePosition getPosition() {
    double distance = inputs.driveDistanceMeters;
    Rotation2d angle = Rotation2d.fromDegrees(inputs.anglePositionDeg);
    return new SwerveModulePosition(distance, angle);
  }

  /**
   * Get the number of this swerve module.
   *
   * @return the number of this swerve module
   */
  public int getModuleNumber() {
    return moduleNumber;
  }

  /**
   * Get the stator current of the drive motor of this swerve module.
   *
   * @return the stator current of the drive motor of this swerve module
   */
  public double getDriveCurrent() {
    return inputs.driveStatorCurrentAmps;
  }

  /**
   * Update this swerve module's inputs and log them.
   *
   * <p>This method must be invoked by the drivetrain subsystem's periodic method.
   */
  public void updateAndProcessInputs() {
    this.lastAngleMotorVelocity = inputs.angleVelocityRevPerMin;
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Mod" + moduleNumber, inputs);
  }

  /**
   * Set the brake mode of the drive motor.
   *
   * @param enable if true, the drive motor will be set to brake mode; if false, coast mode.
   */
  public void setDriveBrakeMode(boolean enable) {
    io.setDriveBrakeMode(enable);
  }

  /**
   * Set the brake mode of the angle motor.
   *
   * @param enable if true, the angle motor will be set to brake mode; if false, coast mode.
   */
  public void setAngleBrakeMode(boolean enable) {
    io.setAngleBrakeMode(enable);
  }

  /**
   * Get the velocity of the angle motor in radians per second.
   *
   * @return the velocity of the angle motor in radians per second
   */
  public double getAngleMotorVelocity() {
    return inputs.angleVelocityRevPerMin * 2.0 * Math.PI / 60.0;
  }

  /**
   * Get the acceleration of the angle motor in radians per second^2.
   *
   * @return the acceleration of the angle motor in radians per second^2
   */
  public double getAngleMotorAcceleration() {
    return ((inputs.angleVelocityRevPerMin - this.lastAngleMotorVelocity) / 60.0 * 2.0 * Math.PI)
        / LOOP_PERIOD_SECS;
  }

  /**
   * Returns a list of status signals for the swerve module related to odometry. This can be used to
   * synchronize the gyro and swerve modules to improve the accuracy of pose estimation.
   *
   * @return the status signals for the swerve module
   */
  public List<StatusSignal<Double>> getOdometryStatusSignals() {
    return io.getOdometryStatusSignals();
  }
}
