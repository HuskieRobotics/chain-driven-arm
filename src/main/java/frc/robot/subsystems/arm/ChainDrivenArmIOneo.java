package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ChainDrivenArmConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.swerve.Conversions;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.lib.team6328.util.TunableNumber;

/** TalonFX implementation of the generic SubsystemIO */
public class ChainDrivenArmIOneo implements ChainDrivenArmIO {
  private CANSparkMax motorOne;
  private CANSparkMax motorTwo;
  private CANSparkMax motorThree;
  private CANSparkMax motorFour;
  private RelativeEncoder m_altEncoder;
  private SparkMaxPIDController m_pidController;

  private Alert configAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.ERROR);

  private final TunableNumber kP = new TunableNumber("Subsystem/kP", POSITION_PID_P);
  private final TunableNumber kI = new TunableNumber("Subsystem/kI", POSITION_PID_I);
  private final TunableNumber kD = new TunableNumber("Subsystem/kD", POSITION_PID_D);
  private final TunableNumber kPeakOutput =
      new TunableNumber("Subsystem/kPeakOutput", POSITION_PID_PEAK_OUTPUT);

  /** Create a TalonFX-specific generic SubsystemIO */
  public ChainDrivenArmIOneo() {
    this.m_altEncoder = motorOne.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, COUNTS_PER_REV); // FIXME: 2nd param is using gear ratio for counts per rev
    this.m_pidController.setFeedbackDevice(m_altEncoder);

    configMotors(MOTOR_ONE_ID, MOTOR_TWO_ID, MOTOR_THREE_ID, MOTOR_FOUR_ID);
  }

  /**
   * Update the inputs based on the current state of the TalonFX motor controller.
   *
   * @param inputs the inputs object to update
   */
  @Override
  public void updateInputs(ChainDrivenArmIOInputsAutoLogged inputs) { // FIXME: why is this an error?
    inputs.positionDeg = m_altEncoder.getPosition();                         // positionDeg
    inputs.velocityRPM = m_altEncoder.getVelocity();                         // velocityRPM
    inputs.closedLoopError = m_altEncoder.getPosition() - inputs.setpoint;   // closedLoopError
    inputs.power = motorOne.getAppliedOutput();                              // power
    inputs.controlMode = motorOne.getMotorType().toString();                 // motorType   0=brushed, 1=brushless
    inputs.supplyCurrentAmps = motorOne.getOutputCurrent();                  // current/Amps
    inputs.tempCelsius = motorOne.getMotorTemperature();                     // temp
    

    // update configuration if tunables have changed
    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged() || kPeakOutput.hasChanged()) {
      this.m_pidController = motorOne.getPIDController();
      this.m_pidController.setFeedbackDevice(m_altEncoder);

      m_pidController.setP(kP.get());
      m_pidController.setI(kI.get());
      m_pidController.setD(kD.get());
      m_pidController.setFF(0); 
      m_pidController.setOutputRange(-kPeakOutput.get(), kPeakOutput.get());
      
    }
  }

  /**
   * Set the motor power to the specified percentage of maximum power.
   *
   * @param power the percentage of maximum power to set the motor to
   */
  @Override
  public void setMotorPower(double power) {
    this.motorOne.set(power);
  }

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   * @param arbitraryFeedForward the arbitrary feed forward as a percentage of maximum power
   */
  @Override
  public void setMotorPosition(double position) {
    this.m_pidController.setReference(position, ControlType.kPosition);
  }

  @Override
  public boolean atPosition(){
    return (-1 <= m_altEncoder.getPosition()) && m_altEncoder.getPosition() <= 1;
  }

  
  private void configMotors(int motorOneID, int motorTwoID, int motorThreeID, int motorFourID) {

    
    // FIXME: need to eventually support both pairs of motors

    // Motor Group 1; Drive Clockwise
    this.motorOne = new CANSparkMax(motorOneID, MotorType.kBrushless);
    this.motorTwo = new CANSparkMax(motorTwoID, MotorType.kBrushless);
    this.motorTwo.follow(motorOne);
    // Motor Group 2; Drive Counter-clockwise
    this.motorThree = new CANSparkMax(motorThreeID, MotorType.kBrushless);
    this.motorFour = new CANSparkMax(motorTwoID, MotorType.kBrushless);
    this.motorThree.follow(motorOne, true); // Mirrors the voltage of motorOne
    this.motorFour.follow(motorThree); 

    m_pidController.setP(kP.get());
    m_pidController.setI(kI.get());
    m_pidController.setD(kD.get());
    m_pidController.setFF(
        0); // FIXME: later, this should be set to a value proportional to the cosine of the angle
    m_pidController.setOutputRange(-kPeakOutput.get(), kPeakOutput.get());
    // below is original

    this.motorOne.setSmartCurrentLimit(
        CONTINUOUS_CURRENT_LIMIT); 

    motorOne.setIdleMode(MODE);
    
  }
}
