package edu.wpi.first.math.geometry;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class Ellipse2dTest {
  @Test
  void testGetFocalPoints() {
    var center = new Pose2d(1, 2, new Rotation2d());
    var ellipse = new Ellipse2d(center, 5.0, 4.0);

    var a = ellipse.getFocalPoint(true);
    var b = ellipse.getFocalPoint(false);

    assertAll(
      () -> assertEquals(new Translation2d(-2.0, 2.0), a),
      () -> assertEquals(new Translation2d(4.0, 2.0), b));
  }

  @Test
  void testIntersectsPoint() {
    var center = new Pose2d(1.0, 2.0, new Rotation2d());
    var ellipse = new Ellipse2d(center, 2.0, 1.0);

    var pointA = new Translation2d(1.0, 3.0);
    var pointB = new Translation2d(0.0, 3.0);

    assertAll(
      () -> assertTrue(ellipse.intersectsPoint(pointA)),
      () -> assertFalse(ellipse.intersectsPoint(pointB)));
  }

  @Test
  void testContainsPoint() {
    var center = new Pose2d(-1.0, -2.0, Rotation2d.fromDegrees(45.0));
    var ellipse = new Ellipse2d(center, 2.0, 1.0);

    var pointA = new Translation2d(0.0, -1.0);
    var pointB = new Translation2d(0.5, -2.0);

    assertAll(
      () -> assertTrue(ellipse.containsPoint(pointA)),
      () -> assertFalse(ellipse.containsPoint(pointB)));
  }

  @Test
  void testEquals() {
    var center1 = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0));
    var ellipse1 = new Ellipse2d(center1, 2.0, 3.0);

    var center2 = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0));
    var ellipse2 = new Ellipse2d(center1, 2.0, 3.0);

    var center3 = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(90.0));
    var ellipse3 = new Ellipse2d(center1, 3.0, 2.0);

    assertAll(
      () -> assertTrue(ellipse1.equals(ellipse2)),
      () -> assertFalse(ellipse1.equals(ellipse3)),
      () -> assertFalse(ellipse3.equals(ellipse2)));
  }
}
