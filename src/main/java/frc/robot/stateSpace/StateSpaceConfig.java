// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.stateSpace;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.system.LinearSystem;
import frc.robot.Constants.StateSpaceConstants;

public class StateSpaceConfig<States extends Num, Inputs extends Num, Outputs extends Num> {
  private LinearSystem<States, Inputs, Outputs> m_plant;
  private Vector<States> m_stateStdDevs;
  private Vector<Outputs> m_outputStdDevs;

  private Vector<States> m_qelms;
  private Vector<Inputs> m_relms;
  
  private Nat<States> m_states;
  private Nat<Outputs> m_outputs;

  private String m_name;
  /** Creates a new StateSpaceConfig. */
  public StateSpaceConfig(
    LinearSystem<States, Inputs, Outputs> plant,
    Vector<States> stateStdDevs,
    Vector<Outputs> outputStdDevs,
    Vector<States> qelms,
    Vector<Inputs> relms,
    Nat<States> states,
    Nat<Outputs> outputs,
    String name
    ) {

      m_plant = plant;
      
      m_stateStdDevs = stateStdDevs;
      m_outputStdDevs = outputStdDevs;

      m_qelms = qelms;
      m_relms = relms;

      m_states = states;
      m_outputs = outputs;

      m_name = name;
    }

  public KalmanFilter<States, Inputs, Outputs> buildObserver() {
    return new KalmanFilter<States, Inputs, Outputs>(
      m_states,
      m_outputs,
      m_plant,
      m_stateStdDevs,
      m_outputStdDevs,
      StateSpaceConstants.k_dt
    );
  }

  public LinearQuadraticRegulator<States, Inputs, Outputs> buildController() {
    return new LinearQuadraticRegulator<States, Inputs, Outputs>(
      m_plant,
      m_qelms,
      m_relms,
      StateSpaceConstants.k_dt
    );
  }

  public String getName() {
    return m_name;
  }

  public LinearSystem<States, Inputs, Outputs> getPlant() {
    return m_plant;
  }
}
