// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.TalonFXConstants;
import frc.robot.subsystems.controllers.PositionStateSpaceController;

public class Arm extends SubsystemBase {

  private TalonFX shoulderMotor;
  private TalonFX wristMotor;

  private PositionStateSpaceController shoulderController;
  private PositionStateSpaceController wristController;

  private CANcoder shoulderCancoder;
  private CANcoder wristCancoder;

  /** Creates a new Arm. */
  public Arm() {
    this.shoulderMotor = new TalonFX(ArmConstants.shoulderMotorPort);
    this.wristMotor = new TalonFX(ArmConstants.wristMotorPort);

    this.shoulderCancoder = new CANcoder(ArmConstants.shoulderCancoderPort);
    this.wristCancoder = new CANcoder(ArmConstants.wristCancoderPort);

    this.shoulderController = new PositionStateSpaceController(ArmConstants.shoulderMomentOfIntertia, ArmConstants.shoulderGearRatio, TalonFXConstants.TalonFXDCMotor, this::getShoulderPosition, this::applyShoulderVoltage, ArmConstants.shoulderName);
    this.wristController = new PositionStateSpaceController(ArmConstants.wristMomentOfIntertia, ArmConstants.wristGearRatio, TalonFXConstants.TalonFXDCMotor, this::getWristPosition, this::applyWristVoltage, ArmConstants.wristName);
  }

  private double getShoulderPosition() {
    return shoulderCancoder.getAbsolutePosition().getValueAsDouble();
  }
  private double getWristPosition() {
    return wristCancoder.getAbsolutePosition().getValueAsDouble();
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