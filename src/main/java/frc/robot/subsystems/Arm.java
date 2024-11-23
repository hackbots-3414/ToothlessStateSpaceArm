// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.stateSpace.StateSpaceController;

public class Arm extends SubsystemBase {

  private TalonFX m_shoulderMotor;
  private TalonFX m_wristMotor;

  private CANcoder m_shoulderCancoder;
  private CANcoder m_wristCancoder;

  private StateSpaceController<N2, N1, N1> m_shoulderController;
  private StateSpaceController<N2, N1, N1> m_wristController;

  /** Creates a new Arm. */
  public Arm() {
    m_shoulderMotor = new TalonFX(ShoulderConstants.shoulderMotorPort);
    m_wristMotor = new TalonFX(WristConstants.wristMotorPort);

    m_shoulderCancoder = new CANcoder(ShoulderConstants.shoulderCancoderPort);
    m_wristCancoder = new CANcoder(WristConstants.wristCancoderPort);

    Vector<N2> initialShoulderState = VecBuilder.fill(
      m_shoulderCancoder.getAbsolutePosition().getValueAsDouble(),
      m_shoulderCancoder.getVelocity().getValueAsDouble()
    );

    Vector<N2> initialWristState = VecBuilder.fill(
      m_wristCancoder.getAbsolutePosition().getValueAsDouble(),
      m_shoulderCancoder.getVelocity().getValueAsDouble()
    );

    m_shoulderController = new StateSpaceController<N2, N1, N1>(
      ShoulderConstants.k_shoulderControllerConfig,
      this::getShoulderPosition,
      this::applyShoulderVoltage,
      initialShoulderState
    );

    m_wristController = new StateSpaceController<N2, N1, N1>(
      WristConstants.k_shoulderControllerConfig,
      this::getWristPosition,
      this::applyWristVoltage,
      initialWristState
    );
  }

  private Vector<N1> getShoulderPosition() {
    return VecBuilder.fill(
      m_shoulderCancoder.getAbsolutePosition().getValueAsDouble()
    );
  }
  private Vector<N1> getWristPosition() {
    return VecBuilder.fill(
      m_wristCancoder.getAbsolutePosition().getValueAsDouble()
    );
  }

  private void applyShoulderVoltage(Vector<N1> input) {
    double volts = input.get(0);
    m_shoulderMotor.setVoltage(volts);
  }

  private void applyWristVoltage(Vector<N1> input) {
    double volts = input.get(0);
    m_wristMotor.setVoltage(volts);
  }

  public void setState(double shoulderPosition, double wristPosition) {
    m_shoulderController.setReference(VecBuilder.fill(shoulderPosition, 0.0));
    m_wristController.setReference(VecBuilder.fill(shoulderPosition, 0.0));
  }
}