// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.autodiff;

import edu.wpi.first.math.jni.AutodiffJNI;
import edu.wpi.first.util.WPICleaner;
import java.lang.ref.Cleaner.Cleanable;
import java.lang.ref.WeakReference;
import java.util.concurrent.ConcurrentHashMap;

/** An autodiff variable pointing to an expression node. */
public class Variable implements AutoCloseable {
  private static final ConcurrentHashMap<Long, WeakReference<Variable>> cache =
      new ConcurrentHashMap<>();

  // TODO equality, inequalities
  private long m_impl;
  private final Cleanable m_cleanable;

  /**
   * Constructs a variable object with the given handle.
   *
   * @param handle The implementation handle of the variable.
   */
  @SuppressWarnings("this-escape")
  private Variable(long impl) {
    m_impl = impl;
    m_cleanable = WPICleaner.register(this, this::cleanup);
  }

  /**
   * Constructs a variable object containing a constant value.
   *
   * @param value The value of the variable.
   * @return The variable.
   */
  public static Variable fromConstant(double value) {
    return new Variable(AutodiffJNI.createConstantVariable(value));
  }

  /**
   * Creates a new variable from a native implementation handle, caching it to ensure that two
   * variable objects don't have the same implementation handle.
   *
   * @param impl Implementation handle of the variable.
   * @return The variable.
   */
  protected static Variable fromHandle(long impl) {
    WeakReference<Variable> ref = cache.get(impl);
    Variable v = (ref != null) ? ref.get() : null;

    if (v == null) {
      v = new Variable(impl);
      cache.put(impl, new WeakReference<>(v));
    }

    return v;
  }

  /**
   * Returns the product of this variable and another.
   *
   * @param multiplier The variable to multiply by.
   * @return The product.
   */
  public Variable times(Variable multiplier) {
    return new Variable(AutodiffJNI.multiplyVariables(m_impl, multiplier.getHandle()));
  }

  /**
   * Returns the product of this variable and a constant.
   *
   * @param multiplier The constant multiplier.
   * @return The product.
   */
  public Variable times(double multiplier) {
    return times(fromConstant(multiplier));
  }

  /**
   * Returns the quotient of this variable and another.
   *
   * @param divisor The variable to divide by.
   * @return The quotient.
   */
  public Variable div(Variable divisor) {
    return new Variable(AutodiffJNI.divideVariables(m_impl, divisor.getHandle()));
  }

  /**
   * Returns the quotient of this variable and a constant.
   *
   * @param divisor The constant to divide by.
   * @return The quotient.
   */
  public Variable div(double divisor) {
    return div(fromConstant(divisor));
  }

  /**
   * Returns the sum of this variable and another.
   *
   * @param other The variable to add.
   * @return The sum.
   */
  public Variable plus(Variable other) {
    return new Variable(AutodiffJNI.addVariables(m_impl, other.getHandle()));
  }

  /**
   * Returns the sum of this variable and a constant.
   *
   * @param other The constant to add.
   * @return The sum.
   */
  public Variable plus(double other) {
    return plus(fromConstant(other));
  }

  /**
   * Returns the difference of this variable and another.
   *
   * @param other The variable to subtract.
   * @return The difference.
   */
  public Variable minus(Variable other) {
    return new Variable(AutodiffJNI.subtractVariables(m_impl, other.getHandle()));
  }

  /**
   * Returns the difference of this variable and a constant.
   *
   * @param other The constant to subtract.
   * @return The difference.
   */
  public Variable minus(double other) {
    return minus(fromConstant(other));
  }

  /**
   * Returns this variable raised to a power.
   *
   * @param exp The exponent to raise this variable to.
   * @return A variable representing {@code this ^ exp}.
   */
  public Variable pow(Variable exp) {
    return new Variable(AutodiffJNI.powerVariables(m_impl, exp.getHandle()));
  }

  /**
   * Returns this variable raised to a constant power.
   *
   * @param other The exponent to raise this variable to.
   * @return A variable representing {@code this ^ exp}.
   */
  public Variable pow(double exp) {
    return pow(fromConstant(exp));
  }

  /**
   * Returns the negation of this variable.
   *
   * @return The negation of this variable.
   */
  public Variable neg() {
    return times(-1);
  }

  /**
   * Gets the value of this variable.
   *
   * @return The value of this variable.
   */
  public double getValue() {
    return AutodiffJNI.getVariableValue(m_impl);
  }

  /**
   * Gets the type of this expression (constant, linear, quadratic, or nonlinear).
   *
   * @return The type of this expression.
   */
  public ExpressionType getType() {
    return ExpressionType.fromValue(AutodiffJNI.getVariableType(m_impl));
  }

  /**
   * Gets this variable's implementation handle.
   *
   * @return The implementation handle, or 0 if it has been closed.
   */
  public long getHandle() {
    return m_impl;
  }

  @Override
  public void close() {
    m_cleanable.clean();
  }

  private void cleanup() {
    AutodiffJNI.freeVariable(m_impl);
    cache.remove(m_impl);
    m_impl = 0;
  }
}
