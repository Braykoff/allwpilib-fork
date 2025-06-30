// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.autodiff;

import edu.wpi.first.math.jni.AutodiffJNI;

/** A matrix of autodiff variables. */
public class VariableMatrix {
  private long m_impl;

  /**
   * Returns a variable matrix filled with zeroes.
   *
   * @param rows The number of matrix rows.
   * @param cols The number of matrix columns.
   * @return A variable matrix filled with zeroes.
   */
  public static VariableMatrix zero(int rows, int cols) {
    return null;
  }

  public static VariableMatrix ones(int rows, int cols) {
    return null;
  }

  /**
   * Constructs a variable matrix with the given implementation handle.
   *
   * @param impl The implementation handle.
   */
  private VariableMatrix(long impl) {
    m_impl = impl;
  }

  /**
   * Constructs a variable matrix with the given number of rows and columns.
   *
   * @param rows The number of matrix rows.
   * @param cols The number of matrix columns.
   */
  public VariableMatrix(int rows, int cols) {
    this(AutodiffJNI.createVariableMatrix(rows, cols));
  }

  /**
   * Constructs a VariableMatrix column vector with the given rows.
   *
   * @param rows The number of matrix rows.
   */
  public VariableMatrix(int rows) {
    this(rows, 1);
  }
}
