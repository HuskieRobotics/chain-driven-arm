// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ChainDrivenArm;

public class ArmToPose extends CommandBase {
  /** Creates a new ArmToPose. */

  protected ChainDrivenArm arm;
  private double angle; // degrees

  public ArmToPose(ChainDrivenArm subsystem, double angleDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angle = angleDegrees;
    this.arm = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Logger.getInstance().recordOutput("ActiveCommands/SetArmPose", true);
    arm.setMotorPosition(this.angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.arm.setMotorPosition(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isAtPosition();
  }
}
