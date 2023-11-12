package frc.robot.subsystems.chainArm;

import static frc.robot.subsystems.subsystem.SubsystemConstants.*;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.hal.CANAPIJNI;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve.Conversions;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.robot.commands.FeedForwardCharacterization;
import frc.lib.team6328.util.TunableNumber;

/** TalonFX implementation of the generic SubsystemIO */
public class ChainDrivenArmIOneo implements ChainDrivenArmIO {
  private CANSparkMax motor;
  private CANSparkMax.ControlType controlType;

  private VoltageOut voltageRequest;
  private TorqueCurrentFOC currentRequest;
  private PositionVoltage positionRequest;

  private Alert configAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.ERROR);

  private final TunableNumber kP = new TunableNumber("Subsystem/kP", POSITION_PID_P);
  private final TunableNumber kI = new TunableNumber("Subsystem/kI", POSITION_PID_I);
  private final TunableNumber kD = new TunableNumber("Subsystem/kD", POSITION_PID_D);
  private final TunableNumber kPeakOutput =
      new TunableNumber("Subsystem/kPeakOutput", POSITION_PID_PEAK_OUTPUT);

  /** Create a TalonFX-specific generic SubsystemIO */
  public ChainDrivenArmIOneo() {
    configMotor(MOTOR_CAN_ID);
  }

  /**
   * Update the inputs based on the current state of the TalonFX motor controller.
   *
   * @param inputs the inputs object to update
   */
  @Override
  public void updateInputs(ChainDrivenArmIOInputs inputs) {
    inputs.positionRad =
        Conversions.neoRotationsToMechanismRadians(motor.getEncoder().getPosition(), GEAR_RATIO);
    inputs.velocityRPM = motor.getEncoder().getVelocity();
    inputs.setpoint = motor.getClosedLoopReference().getValue(); // fixme Not sure what this is supposed to do
    inputs.closedLoopError = motor.getEncoder().getPosition() - inputs.setpoint;
    inputs.power = motor.getAppliedOutput();
    inputs.controlMode = motor.getMotorType().toString();
    inputs.statorCurrentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();

    // update configuration if tunables have changed
    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged() || kPeakOutput.hasChanged()) {
      SparkMaxPIDController m_pidController = motor.getPIDController();
      
      m_pidController.setP(kP.get());
      m_pidController.setI(kI.get());
      m_pidController.setD(kD.get());
      m_pidController.setFF(0); // FIXME What is the proper value?
      m_pidController.setOutputRange(-kPeakOutput.get(), kPeakOutput.get());
      
      this.motor.getConfigurator().apply(config);
    }
  }

  /**
   * Set the motor power to the specified percentage of maximum power.
   *
   * @param power the percentage of maximum power to set the motor to
   */
  @Override
  public void setMotorPower(double power) {
    this.motor.set(power);
  }

  /**
   * Set the motor current to the specified value in amps.
   *
   * @param power the current to set the motor to in amps
   */
  @Override
  public void setMotorCurrent(double current) {
    this.motor.set(currentRequest.withOutput(current));
  }

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   * @param arbitraryFeedForward the arbitrary feed forward as a percentage of maximum power
   */
  @Override
  public void setMotorPosition(double position, double arbitraryFeedForward) {
    this.motor.setControl(
        positionRequest
            .withPosition(Conversions.degreesToFalconRotations(position, GEAR_RATIO))
            .withFeedForward(arbitraryFeedForward));
  }

  private void configMotor(int motorID) {

    this.motor = new CANSparkMax(motorID, MotorType.kBrushless);

    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = CONTINUOUS_CURRENT_LIMIT;
    currentLimits.SupplyCurrentThreshold = PEAK_CURRENT_LIMIT;
    currentLimits.SupplyTimeThreshold = PEAK_CURRENT_DURATION;
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    config.MotorOutput.Inverted =
        MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();

    config.Voltage.PeakForwardVoltage = kPeakOutput.get();
    config.Voltage.PeakReverseVoltage = kPeakOutput.get();

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = this.motor.getConfigurator().apply(config);
      if (status.isOK()) {
        configAlert.set(false);
        break;
      }
    }
    if (!status.isOK()) {
      configAlert.set(true);
      configAlert.setText(status.toString());
    }

    this.motor.setRotorPosition(0);

    this.voltageRequest = new VoltageOut(0.0);
    this.voltageRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
    this.currentRequest = new TorqueCurrentFOC(0.0);
    this.positionRequest = new PositionVoltage(0.0).withSlot(0);
    this.positionRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
  }
}
