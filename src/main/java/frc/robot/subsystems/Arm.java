// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.controllers.PositionStateSpaceController;

public class Arm extends SubsystemBase {

  private TalonFX shoulderMotor;
  private TalonFX wristMotor;

  private PositionStateSpaceController shoulderController;
  private PositionStateSpaceController wristController;

  /** Creates a new Arm. */
  public Arm() {
    this.shoulderMotor = new TalonFX(ArmConstants.shoulderMotorPort);
    this.wristMotor = new TalonFX(ArmConstants.wristMotorPort);

    this.shoulderController = new PositionStateSpaceController(ArmConstants.shoulderKv, ArmConstants.shoulderKa, this::getShoulderPosition, this::applyShoulderVoltage, ArmConstants.shoulderName);
    this.wristController = new PositionStateSpaceController(ArmConstants.wristKv, ArmConstants.wristKa, this::getWristPosition, this::applyWristVoltage, ArmConstants.wristName);
  }

  private double getShoulderPosition() {
    return shoulderMotor.getPosition().getValueAsDouble();
  }
  private double getWristPosition() {
    return wristMotor.getPosition().getValueAsDouble();
  }

  private void applyShoulderVoltage(double volts) {
    shoulderMotor.setVoltage(volts);
  }

  private void applyWristVoltage(double volts) {
    wristMotor.setVoltage(volts);
  }

  public void setState(double shoulderPosition, double wristPosition) {
    shoulderController.setPosition(shoulderPosition);
    wristController.setPosition(wristPosition);
  }
}