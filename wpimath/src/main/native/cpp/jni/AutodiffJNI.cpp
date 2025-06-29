// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <jni.h>

#include <wpi/jni_util.h>

#include "edu_wpi_first_math_jni_AutodiffJNI.h"
#include <sleipnir/autodiff/variable.hpp>

using namespace wpi::java;

extern "C" {

/*
 * Class:     edu_wpi_first_math_jni_AutodiffJNI
 * Method:    createConstantVariable
 * Signature: (D)J
 */
JNIEXPORT jlong JNICALL
Java_edu_wpi_first_math_jni_AutodiffJNI_createConstantVariable
  (JNIEnv *, jclass, jdouble value)
{
  return reinterpret_cast<jlong>(new slp::Variable(value));
}

}  // extern "C"
