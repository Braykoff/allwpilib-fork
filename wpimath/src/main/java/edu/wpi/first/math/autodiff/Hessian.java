// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.autodiff;

import org.ejml.data.DMatrixSparseCSC;

/**
 * This class calculates the Hessian of a variable with respect to a vector of variables.
 *
 * <p>The gradient tree is cached so subsequent Hessian calculations are faster, and the Hessian is
 * only recomputed if the variable expression is nonlinear.
 */
public class Hessian {
  /** Represents which part of the Hessian to compute. */
  public static enum UpLo {
    LOWER,
    BOTH
  }

  /**
   * Constructs a Hessian object.
   *
   * @param variable Variable of which to compute the Hessian.
   * @param wrt Variable with respect to which to compute the Hessian.
   * @param part Which part of the Hessian to compute.
   */
  public Hessian(Variable variable, VariableMatrix wrt, UpLo part) {}

  /**
   * Returns the Hessian as a VariableMatrix.
   *
   * <p>This is useful when constructing optimization problems with derivatives in them.
   *
   * @return The Hessian as a VariableMatrix.
   */
  public VariableMatrix get() {
    return null;
  }

  /**
   * Evaluates the Hessian at wrt's value.
   *
   * @return The Hessian at wrt's value.
   */
  public DMatrixSparseCSC value() {
    return null;
  }
}
