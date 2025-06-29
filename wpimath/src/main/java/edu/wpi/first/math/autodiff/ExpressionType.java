// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.autodiff;

import edu.wpi.first.math.jni.AutodiffJNI;

/** Expression type. Used for autodiff caching. */
public enum ExpressionType {
  NONE(AutodiffJNI.EXPRESSION_TYPE_NONE),
  CONSTANT(AutodiffJNI.EXPRESSION_TYPE_CONSTANT),
  LINEAR(AutodiffJNI.EXPRESSION_TYPE_LINEAR),
  QUADRATIC(AutodiffJNI.EXPRESSION_TYPE_QUADRATIC),
  NONLINEAR(AutodiffJNI.EXPRESSION_TYPE_NONLINEAR);

  public final int value;

  private ExpressionType(int value) {
    this.value = value;
  }

  /***
   * Gets an expression type from an int value.
   * @param value Int value
   * @return Expression type
   */
  public static ExpressionType fromValue(int value) {
    return switch (value) {
      case AutodiffJNI.EXPRESSION_TYPE_NONE -> NONE;
      case AutodiffJNI.EXPRESSION_TYPE_CONSTANT -> CONSTANT;
      case AutodiffJNI.EXPRESSION_TYPE_LINEAR -> LINEAR;
      case AutodiffJNI.EXPRESSION_TYPE_QUADRATIC -> QUADRATIC;
      case AutodiffJNI.EXPRESSION_TYPE_NONLINEAR -> NONLINEAR;
      default -> NONE;
    };
  }
}
