// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.jni;

/** Autodiff JNI */
public final class AutodiffJNI extends WPIMathJNI {
  public static final int EXPRESSION_TYPE_NONE = 0;
  public static final int EXPRESSION_TYPE_CONSTANT = 1;
  public static final int EXPRESSION_TYPE_LINEAR = 2;
  public static final int EXPRESSION_TYPE_QUADRATIC = 3;
  public static final int EXPRESSION_TYPE_NONLINEAR = 4;

  /**
   * Creates a variable representing a constant value and returns its implementation handle.
   *
   * @param value The value of the constant.
   * @return Variable implementation handle.
   */
  public static native long createConstantVariable(double value);

  /**
   * Creates a variable representing the product of two variables and returns its implementation
   * handle.
   *
   * @param x Implementation handle of the first variable.
   * @param y Implementation handle of the second variable.
   * @return Implementation handle of the product.
   */
  public static native long multiplyVariables(long x, long y);

  /**
   * Creates a variable representing the quotient of two variables and returns its implementation
   * handle.
   *
   * @param dividend Implementation handle of the dividend variable.
   * @param divisor Implementation handle of the divisor variable.
   * @return Implementation handle of the quotient.
   */
  public static native long divideVariables(long dividend, long divisor);

  /**
   * Creates a variable representing the sum of two variables and returns its implementation handle.
   *
   * @param x Implementation handle of the first variable.
   * @param y Implementation handle of the second variable.
   * @return Implementation handle of the sum.
   */
  public static native long addVariables(long x, long y);

  /**
   * Creates a variable representing the difference of two variables and returns its implementation
   * handle.
   *
   * @param x Implementation handle of the first variable.
   * @param y Implementation handle of the variable to subtract.
   * @return Implementation handle of the difference.
   */
  public static native long subtractVariables(long x, long y);

  /**
   * Creates a variable representing the a variable raised to a variable and returns its
   * implementation handle.
   *
   * @param base Implementation handle of the base variable.
   * @param exp Implementation handle of the power variable.
   * @return Implementation handle of the base raised to the exp.
   */
  public static native long powerVariables(long base, long exp);

  /**
   * Gets the value of a variable
   *
   * @param impl The implementation handle of the variable.
   * @return The value of the variable.
   */
  public static native double getVariableValue(long impl);

  /**
   * Gets the expression type of a variable
   *
   * @param impl The implementation handle of the variable.
   * @return The expression type of the variable.
   */
  public static native int getVariableType(long impl);
}
