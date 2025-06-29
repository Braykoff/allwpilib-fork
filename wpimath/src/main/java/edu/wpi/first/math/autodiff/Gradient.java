// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.autodiff;

import org.ejml.data.DMatrixSparseCSC;

/**
 * This class calculates the gradient of a variable with respect to a vector of variables.
 *
 * <p>The gradient is only recomputed if the variable expression is quadratic or higher order.
 */
public class Gradient {
  /**
   * Constructs a Gradient object.
   *
   * @param variable Variable of which to compute the gradient.
   * @param wrt Variable with respect to which to compute the gradient.
   */
  public Gradient(Variable variable, Variable wrt) {}

  /**
   * Constructs a Gradient object.
   *
   * @param variable Variable of which to compute the gradient.
   * @param wrt Variable matrix with respect to which to compute the gradient.
   */
  public Gradient(Variable variable, VariableMatrix wrt) {}

  /**
   * Returns the gradient as a VariableMatrix.
   *
   * <p>This is useful when constructing optimization problems with derivatives in them.
   *
   * @return The gradient as a VariableMatrix.
   */
  public VariableMatrix get() {
    return null;
  }

  /**
   * Evaluates the gradient at wrt's value.
   *
   * @return The gradient at wrt's value.
   */
  public DMatrixSparseCSC value() {
    return null;
  }
}
