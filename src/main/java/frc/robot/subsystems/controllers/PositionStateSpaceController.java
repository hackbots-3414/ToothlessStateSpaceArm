// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.controllers;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import javax.swing.text.Position;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PositionStateSpaceController extends SubsystemBase {

    private static final double dt = 0.02;

    private LinearSystem<N2, N1, N1> m_flywheelPlant;
    private KalmanFilter<N2, N1, N1> m_observer;
    private LinearQuadraticRegulator<N2, N1, N1> m_controller;
    private LinearSystemLoop<N2, N1, N1> m_loop;

    private DoubleSupplier positionSupplier;
    private DoubleConsumer applyVoltage;

    private boolean stopped;

    private String name;

    /** Creates a statespacecontroller */
    public PositionStateSpaceController(LinearSystem<N2, N1, N1> flywheelPlant, DoubleSupplier positionSupplier, DoubleConsumer applyVoltage, String name) {

        this.positionSupplier = positionSupplier;
        this.applyVoltage = applyVoltage;

        this.stopped = false;

        this.name = name;

        this.m_flywheelPlant = flywheelPlant; // LinearSystemId.createSingleJointedArmSystem(dcMotor, momentOfIntertia, gearing)

        // The observer fuses our encoder data and voltage inputs to reject noise.
        this.m_observer = new KalmanFilter<N2, N1, N1>(
                Nat.N2(),
                Nat.N1(),
                m_flywheelPlant,
                VecBuilder.fill(5.0, 1.0), // How accurate we think our model is
                VecBuilder.fill(1 / 2048.0), // How accurate we think our encoder data is
                dt);

        // A LQR uses feedback to create voltage commands.
        this.m_controller = new LinearQuadraticRegulator<>(
                m_flywheelPlant,
                VecBuilder.fill(0.1, 1.0), // State error tolerance
                VecBuilder.fill(12.0), // Control effort (voltage) tolerance
                dt);

        // The state-space loop combines a controller, observer, feedforward and plant
        // for easy control.
        this.m_loop = new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12.0, 0.020);

        m_loop.reset(VecBuilder.fill(positionSupplier.getAsDouble(), 0.0));
    }


    @Override
    public void periodic() {
        // Correct our Kalman filter's state vector estimate with encoder data.
        double position = positionSupplier.getAsDouble();
        m_loop.correct(VecBuilder.fill(position));
        SmartDashboard.putNumber("StateSpace " + name, position);

        // Update our LQR to generate new voltage commands and use the voltages to
        // predict the next
        // state with out Kalman filter.
        m_loop.predict(dt);

        // Send the new calculated voltage to the motors.
        // voltage = duty cycle * battery voltage, so
        // duty cycle = voltage / battery voltage
        double nextVoltage = m_loop.getU(0);
        if (!stopped) applyVoltage.accept(nextVoltage);

    }

    /**
     * Sets the next reference state's position.
     * @param position the position that the state should be driven to. Units of whatever the positionSupplier gives.
     */
    public void setPosition(double position) {
        m_loop.setNextR(VecBuilder.fill(position, 0.0));
        stopped = false;
    }

    /**
     * This method sets the refrence to the current position. This will cause the state to stop changing (and stay even with external inputs)
     * To apply 0 volts to the motor, use hardStop()
     */
    public void stop() {
        m_loop.setNextR(VecBuilder.fill(positionSupplier.getAsDouble(), 0.0)); // if we set the reference to the current position, it will stop and stay in place.
    }


    /**
     * This method sets 0 volts to the motor, and no new control inputs will be applied.
     * Set a new refrence point to disable this and return to regular control.
     */
    public void hardStop() {
        applyVoltage.accept(0.0);
        stopped = true;
    }
}
