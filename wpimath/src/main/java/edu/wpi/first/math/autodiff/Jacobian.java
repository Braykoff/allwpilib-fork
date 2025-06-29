// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.autodiff;

import org.ejml.data.DMatrixSparseCSC;

/**
 * This class calculates the Jacobian of a vector of variables with respect to a vector of
 * variables.
 *
 * <p>The Jacobian is only recomputed if the variable expression is quadratic or higher order.
 */
public class Jacobian {
  /**
   * Constructs a Jacobian object.
   *
   * @param variable Variable of which to compute the Jacobian.
   * @param wrt Variable with respect to which to compute the Jacobian.
   */
  public Jacobian(VariableMatrix variable, VariableMatrix wrt) {}

  /**
   * Returns the Jacobian as a VariableMatrix.
   *
   * <p>This is useful when constructing optimization problems with derivatives in them.
   *
   * @return The Jacobian as a VariableMatrix.
   */
  public VariableMatrix get() {
    return null;
  }

  /**
   * Evaluates the Jacobian at wrt's value.
   *
   * @return The Jacobian at wrt's value.
   */
  public DMatrixSparseCSC value() {
    return null;
  }
}
