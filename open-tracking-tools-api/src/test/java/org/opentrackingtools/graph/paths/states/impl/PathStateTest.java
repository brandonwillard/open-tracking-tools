package org.opentrackingtools.graph.paths.states.impl;

import org.testng.annotations.Test;
import org.testng.AssertJUnit;
import org.testng.annotations.BeforeMethod;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.stub;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;

import java.util.List;

import org.geotools.geometry.jts.JTSFactoryFinder;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.edges.InferredEdge;
import org.opentrackingtools.graph.edges.impl.SimpleInferredEdge;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.impl.SimplePathEdge;
import org.opentrackingtools.graph.paths.impl.SimpleInferredPath;
import org.opentrackingtools.graph.paths.impl.TrackingTestUtils;
import org.opentrackingtools.graph.paths.states.impl.SimplePathState;
import org.opentrackingtools.graph.paths.util.PathUtils;

import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

public class PathStateTest {

  private InferredPath p1;
  private InferredPath p1_neg;
  private InferredPath p2;
  private InferredPath p2_neg;
  private InferredPath p2_rev;
  
  private InferredEdge ie2_rev;
  private InferredEdge ie3_rev;
  private InferredEdge ie3;
  private InferredEdge ie2;
  private InferredEdge ie1;
  private InferredEdge ie1_rev;
  
  private final double _numericError = 1e-4;
  
  private InferenceGraph graph;

  private InferredPath makeTmpPath(boolean isBackward,
    Coordinate... coords) {
    return TrackingTestUtils.makeTmpPath(this.graph,
        isBackward, coords);
  }

  @Test(expectedExceptions = IllegalArgumentException.class)
  public void testBadPathError() {
    final List<SimplePathEdge> edges2_rev =
        Lists.newArrayList(SimplePathEdge.getEdge(ie2_rev, 0,
            false), SimplePathEdge.getEdge(ie3_rev,
            ie2_rev.getLength(), false));

    SimpleInferredPath.getInferredPath(edges2_rev, false);

  }

  @Test
  public void testDistanceBetween1() {
    /*
     * States on the same path.
     */
    final SimplePathState x =
        SimplePathState.getPathState(p1, VectorFactory
            .getDenseDefault().createVector2D(5d, 1d));
    final SimplePathState y =
        SimplePathState.getPathState(p1, VectorFactory
            .getDenseDefault().createVector2D(7d, 2d));
    final Vector diff = y.minus(x);
    AssertJUnit.assertEquals("distance", 2d, diff.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("velocity", 1d, diff.getElement(1),
        _numericError);

    /*
     * Check for symmetry
     */
    final Vector diff2 = x.minus(y);
    AssertJUnit.assertEquals("rev diff", diff.scale(-1), diff2);
  }

  @Test(enabled = false)
  public void testDistanceBetween10() {
    /* FIXME!
     * Test double overlaps: overlaps on the first
     * edge, then doesn't overlap, then overlaps 
     * at the end.
     */

    final InferredPath path1 =
        makeTmpPath(false, new Coordinate(0, 10),
            new Coordinate(0, 0), new Coordinate(10, 0),
            new Coordinate(10, 10), new Coordinate(20, 10));

    final InferredPath path2 =
        makeTmpPath(false, new Coordinate(20, 10),
            new Coordinate(10, 10), new Coordinate(0, 10),
            new Coordinate(0, 0));

    final SimplePathState x =
        SimplePathState.getPathState(path1, VectorFactory
            .getDenseDefault().createVector2D(35d, 1d));
    final SimplePathState y =
        SimplePathState.getPathState(path2, VectorFactory
            .getDenseDefault().createVector2D(5d, 1d));
    final Vector diff = y.minus(x);
    AssertJUnit.assertEquals("distance", 0d, diff.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("velocity", 2d, diff.getElement(1),
        _numericError);

    final Vector diff2 = x.minus(y);
    AssertJUnit.assertEquals("rev distance", 50d, diff2.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("rev velocity", 0d, diff2.getElement(1),
        _numericError);

    final SimplePathState x2 =
        SimplePathState.getPathState(path1, VectorFactory
            .getDenseDefault().createVector2D(22.5d, 1d));
    final SimplePathState y2 =
        SimplePathState.getPathState(path2, VectorFactory
            .getDenseDefault().createVector2D(5d, 1d));
    final Vector diff3 = x2.minus(y2);
    AssertJUnit.assertEquals("distance", 32.5d, diff3.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("velocity", 0d, diff3.getElement(1),
        _numericError);
  }

  @Test(enabled = false)
  public void testDistanceBetween11() {
    /*
     * FIXME!
     */

    final InferredPath path1 =
        makeTmpPath(false, new Coordinate(0, 0),
            new Coordinate(0, 10));

    final InferredPath path2 =
        makeTmpPath(false, new Coordinate(0, 10),
            new Coordinate(0, 0), new Coordinate(-10, 0));

    final SimplePathState x =
        SimplePathState.getPathState(path2, VectorFactory
            .getDenseDefault().createVector2D(15d, 1d));
    final SimplePathState y =
        SimplePathState.getPathState(path1, VectorFactory
            .getDenseDefault().createVector2D(8.5d, 1d));
    final Vector diff = y.minus(x);
    AssertJUnit.assertEquals("distance", -13.5d, diff.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("velocity", -2d, diff.getElement(1),
        _numericError);

    final Vector diff2 = x.minus(y);
    AssertJUnit.assertEquals("rev distance", -13.5d,
        diff2.getElement(0), _numericError);
    AssertJUnit.assertEquals("rev velocity", -2d, diff2.getElement(1),
        _numericError);
  }

  @Test
  public void testDistanceBetween12() {
    final InferredPath startPath =
        makeTmpPath(false, new Coordinate(0, 0),
            new Coordinate(0, 60));

    final SimplePathState startState =
        SimplePathState.getPathState(startPath, VectorFactory
            .getDefault().createVector2D(12d, -1));

    final InferredPath newPath =
        makeTmpPath(true, new Coordinate(0, 60),
            new Coordinate(0, 0));

    final Vector result =
        newPath.getStateOnPath(startState).getRawState();

    final SimplePathState stateOnNewPath =
        SimplePathState.getPathState(newPath, result);
    final Vector difference =
        stateOnNewPath.minus(startState);

    AssertJUnit.assertTrue(difference.isZero(1e-7));
  }

  @Test
  public void testDistanceBetween13() {
    final InferredPath startPath =
        makeTmpPath(false, new Coordinate(0, 0),
            new Coordinate(0, 60), new Coordinate(0, 199));

    final SimplePathState startState =
        SimplePathState.getPathState(startPath, VectorFactory
            .getDefault().createVector2D(104d, -1));

    final InferredPath newPath =
        makeTmpPath(false, new Coordinate(0, 199),
            new Coordinate(0, 60));

    final Vector result =
        newPath.getStateOnPath(startState).getRawState();

    final SimplePathState stateOnNewPath =
        SimplePathState.getPathState(newPath, result);
    final Vector difference =
        stateOnNewPath.minus(startState);

    AssertJUnit.assertTrue(difference.isZero(1e-7));
  }

  @Test
  public void testDistanceBetween14() {
    final InferredPath startPath =
        makeTmpPath(true, new Coordinate(0, 199),
            new Coordinate(0, 60), new Coordinate(0, 0));

    final SimplePathState startState =
        SimplePathState.getPathState(startPath, VectorFactory
            .getDefault().createVector2D(-104d, -2.79));

    final InferredPath newPath =
        makeTmpPath(true, new Coordinate(0, 60),
            new Coordinate(0, 199));

    final Vector result =
        newPath.getStateOnPath(startState).getRawState();

    final SimplePathState stateOnNewPath =
        SimplePathState.getPathState(newPath, result);
    final Vector difference =
        stateOnNewPath.minus(startState);

    AssertJUnit.assertTrue(difference.isZero(1e-7));
  }

  @Test
  public void testDistanceBetween15() {
    final InferredPath startPath =
        makeTmpPath(true, new Coordinate(0,
            178.89095954422947), new Coordinate(0, 0),
            new Coordinate(0, 139.9919743536118d));

    final SimplePathState startState =
        SimplePathState.getPathState(
            startPath,
            VectorFactory.getDefault().createVector2D(
                -139.9919743536118d, -3.39));

    final InferredPath newPath =
        makeTmpPath(false, new Coordinate(0, 0),
            new Coordinate(0, 178.89095954422947));

    final Vector result =
        newPath.getStateOnPath(startState).getRawState();

    final SimplePathState stateOnNewPath =
        SimplePathState.getPathState(newPath, result);
    final Vector difference =
        stateOnNewPath.minus(startState);

    AssertJUnit.assertTrue(difference.isZero(1e-7));
  }

  @Test
  public void testDistanceBetween16() {
    final InferredPath otherPath =
        makeTmpPath(false, new Coordinate(0d, 0d),
            new Coordinate(-85d, 0d), new Coordinate(
                -150.37d, 0d));

    final SimplePathState otherState =
        SimplePathState.getPathState(otherPath, VectorFactory
            .getDefault().createVector2D(121.15d, 2.59d));

    final InferredPath thisPath =
        makeTmpPath(true, new Coordinate(0d, 0d),
            new Coordinate(-85d, 0d), new Coordinate(
                -150.37d, 0d));
    final SimplePathState thisState =
        SimplePathState
            .getPathState(
                thisPath,
                VectorFactory.getDefault().createVector2D(
                    -150.37d, -12.06d));

    final Vector difference = thisState.minus(otherState);

    AssertJUnit.assertEquals("dist", -121d, difference.getElement(0),
        1d);
    AssertJUnit.assertEquals("vel", -14.65d, difference.getElement(1),
        1d);
  }

  @Test
  public void testDistanceBetween17() {
    final InferredPath startPath =
        makeTmpPath(true, new Coordinate(0, -262.21d),
            new Coordinate(0, 0),
            new Coordinate(0, 792.85d));

    final SimplePathState startState =
        SimplePathState.getPathState(
            startPath,
            VectorFactory.getDefault().createVector2D(
                -808.4862233549034, -26.2880263830998));

    final InferredPath newPath =
        makeTmpPath(true, new Coordinate(0, 792.85d),
            new Coordinate(0, 0), new Coordinate(0,
                -262.21d));

    final Vector result =
        newPath.getStateOnPath(startState).getRawState();

    final SimplePathState stateOnNewPath =
        SimplePathState.getPathState(newPath, result);
    final Vector difference =
        stateOnNewPath.minus(startState);

    AssertJUnit.assertTrue(difference.isZero(1e-7));
  }

  @Test
  public void testDistanceBetween18() {
    final InferredPath otherPath =
        makeTmpPath(true, new Coordinate(-90d, 0d),
            new Coordinate(0d, 0d), new Coordinate(0d,
                -(517d - 90d) / 2d),
            new Coordinate(0d, 0d),
            new Coordinate(-90d, 0d));

    final SimplePathState otherState =
        SimplePathState.getPathState(otherPath, VectorFactory
            .getDefault().createVector2D(-564d, -8d));

    final InferredPath thisPath =
        makeTmpPath(false, new Coordinate(0d, 0d),
            new Coordinate(-90d, 0d), new Coordinate(-90d,
                607d - 90d));
    final SimplePathState thisState =
        SimplePathState.getPathState(thisPath, VectorFactory
            .getDefault().createVector2D(47, 8d));

    final Vector difference = thisState.minus(otherState);

    AssertJUnit.assertEquals("dist", 0d, difference.getElement(0), 0d);
    AssertJUnit.assertEquals("vel", 0d, difference.getElement(1), 0d);
  }

  @Test
  public void testDistanceBetween19() {
    /*
     * This path ends at its start (a loop).
     */
    final InferredPath otherPath =
        makeTmpPath(true, new Coordinate(-90d, 0d),
            new Coordinate(0d, 0d), new Coordinate(0d,
                -(517d - 90d) / 2d),
            new Coordinate(0d, 0d),
            new Coordinate(-90d, 0d));

    final SimplePathState otherState =
        SimplePathState.getPathState(otherPath, VectorFactory
            .getDefault().createVector2D(-564d, -8d));

    /*
     * This path starts at the same start as
     * the other. 
     */
    final InferredPath thisPath =
        makeTmpPath(true, new Coordinate(0d, 607d - 90d),
            new Coordinate(0d, 0d),
            new Coordinate(-90d, 0d));

    final Vector result =
        thisPath.getStateOnPath(otherState).getRawState();

    final SimplePathState stateOnThis =
        SimplePathState.getPathState(thisPath, result);
    final Vector difference = stateOnThis.minus(otherState);

    AssertJUnit.assertTrue(difference.isZero(1e-7));
  }

  @Test
  public void testDistanceBetween2() {
    final SimplePathState x =
        SimplePathState.getPathState(p1, VectorFactory
            .getDenseDefault().createVector2D(5d, 1d));
    final SimplePathState y =
        SimplePathState.getPathState(
            p1_neg,
            VectorFactory.getDenseDefault().createVector2D(
                p1_neg.getTotalPathDistance() + 7d, -2d));
    final Vector diff2 = x.minus(y);
    AssertJUnit.assertEquals("distance", -2d, diff2.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("velocity", 3d, diff2.getElement(1),
        _numericError);
  }

  @Test
  public void testDistanceBetween20() {
    final InferredPath otherPath =
        makeTmpPath(true, new Coordinate(0d, 0d),
            new Coordinate(0d, 179d), new Coordinate(10d,
                179d), new Coordinate(56d, 179d));

    final SimplePathState otherState =
        SimplePathState.getPathState(otherPath, VectorFactory
            .getDefault().createVector2D(-41d, 4d));

    final InferredPath thisPath =
        makeTmpPath(false, new Coordinate(0d, 0d),
            new Coordinate(0d, 179d));
    final SimplePathState thisState =
        SimplePathState.getPathState(thisPath, VectorFactory
            .getDefault().createVector2D(157d, 7.5d));

    final Vector difference = thisState.minus(otherState);

    AssertJUnit.assertEquals("dist", -37d, difference.getElement(0), 0d);
    AssertJUnit.assertEquals("vel", 3.5d, difference.getElement(1), 0d);
  }

  @Test
  public void testDistanceBetween21() {
    final InferredPath otherPath =
        makeTmpPath(false, new Coordinate(0d, 0d),
            new Coordinate(100d, 0d));

    final SimplePathState otherState =
        SimplePathState.getPathState(otherPath, VectorFactory
            .getDefault().createVector2D(49d, 1.6d));

    final InferredPath thisPath =
        makeTmpPath(false, new Coordinate(0d, 0d),
            new Coordinate(100d, 0d), new Coordinate(100d,
                100d), new Coordinate(100d, 0d),
            new Coordinate(0d, 0d));
    final SimplePathState thisState =
        SimplePathState.getPathState(thisPath, VectorFactory
            .getDefault().createVector2D(0d, 0d));

    final Vector difference = thisState.minus(otherState);

    AssertJUnit.assertEquals("dist", -49d, difference.getElement(0), 0d);
    AssertJUnit.assertEquals("vel", -1.6d, difference.getElement(1), 0d);
  }

  /**
   * This test checks that a difference from a state
   * on the first edge of a path that overlaps its first
   * edge, but going the opposite direction, at the end
   * will use the first edge only.
   */
  @Test
  public void testDistanceBetween22() {
    final InferredPath otherPath =
        makeTmpPath(false, new Coordinate(0d, 0d),
            new Coordinate(100d, 0d), new Coordinate(100d,
                100d), new Coordinate(100d, 0d),
            new Coordinate(0d, 0d));

    final SimplePathState otherState =
        SimplePathState.getPathState(otherPath, VectorFactory
            .getDefault().createVector2D(49d, 1.6d));

    final InferredPath thisPath =
        makeTmpPath(false, new Coordinate(0d, 0d),
            new Coordinate(100d, 0d));
    final SimplePathState thisState =
        SimplePathState.getPathState(thisPath, VectorFactory
            .getDefault().createVector2D(0d, 0d));

    final Vector difference = thisState.minus(otherState);

    AssertJUnit.assertEquals("dist", -49d, difference.getElement(0), 0d);
    AssertJUnit.assertEquals("vel", -1.6d, difference.getElement(1), 0d);
  }

  @Test(enabled = false, expectedExceptions = IllegalStateException.class)
  public void testDistanceBetween2exp() {
    /*
     * TODO: remove or make relevant again.
     * Same paths, different directions.
     */
    final SimplePathState x =
        SimplePathState.getPathState(p1, VectorFactory
            .getDenseDefault().createVector2D(5d, 1d));
    final SimplePathState y =
        SimplePathState.getPathState(
            p1_neg,
            VectorFactory.getDenseDefault().createVector2D(
                p1_neg.getTotalPathDistance() + 7d, -2d));
    y.minus(x);
  }

  @Test
  public void testDistanceBetween3() {
    /*
     * Both paths are in the same direction, overlapping.
     */
    final SimplePathState x =
        SimplePathState.getPathState(p1, VectorFactory
            .getDenseDefault().createVector2D(12.5d, 1d));
    final SimplePathState y =
        SimplePathState.getPathState(p2, VectorFactory
            .getDenseDefault().createVector2D(17.5d, -1d));
    final Vector diff = y.minus(x);
    AssertJUnit.assertEquals("distance", 15d, diff.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("velocity", -2d, diff.getElement(1),
        _numericError);
  }

  @Test
  public void testDistanceBetween4() {
    final SimplePathState x =
        SimplePathState.getPathState(p1, VectorFactory
            .getDenseDefault().createVector2D(12.5d, 1d));
    final SimplePathState y =
        SimplePathState.getPathState(TrackingTestUtils
            .makeTmpPath(this.graph, true,
                new Coordinate(0, 40),
                new Coordinate(0, 30),
                new Coordinate(0, 20)), VectorFactory
            .getDenseDefault().createVector2D(-17.5d, -1d));
    final Vector diff = y.minus(x);
    AssertJUnit.assertEquals("distance", -15d, diff.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("velocity", 0d, diff.getElement(1),
        _numericError);
  }

  @Test
  public void testDistanceBetween4_2() {
    final SimplePathState x2 =
        SimplePathState.getPathState(TrackingTestUtils
            .makeTmpPath(this.graph, false,
                new Coordinate(0, 30),
                new Coordinate(0, 20),
                new Coordinate(0, 10)), VectorFactory
            .getDenseDefault().createVector2D(12.5d, 1d));
    final SimplePathState y2 =
        SimplePathState.getPathState(p2, VectorFactory
            .getDenseDefault().createVector2D(17.5d, 1d));
    final Vector diff2 = y2.minus(x2);
    AssertJUnit.assertEquals("distance", 20d, diff2.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("velocity", 2d, diff2.getElement(1),
        _numericError);
  }

  @Test
  public void testDistanceBetween4_3() {
    final SimplePathState x2 =
        SimplePathState.getPathState(TrackingTestUtils
            .makeTmpPath(this.graph, false,
                new Coordinate(0, 30),
                new Coordinate(0, 20),
                new Coordinate(0, 10)), VectorFactory
            .getDenseDefault().createVector2D(12.5d, 1d));
    final SimplePathState y2 =
        SimplePathState.getPathState(TrackingTestUtils
            .makeTmpPath(this.graph, true,
                new Coordinate(0, 40),
                new Coordinate(0, 30),
                new Coordinate(0, 20)), VectorFactory
            .getDenseDefault().createVector2D(-17.5d, 1d));
    final Vector diff2 = y2.minus(x2);
    AssertJUnit.assertEquals("distance", -20d, diff2.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("velocity", 0d, diff2.getElement(1),
        _numericError);
  }

  @Test(enabled = false)
  public void testDistanceBetween5() {
    /*
     * FIXME!
     * Both paths are in the same direction, 
     * path2 is the second edge in path1.
     */
    final SimplePathState x =
        SimplePathState.getPathState(p1, VectorFactory
            .getDenseDefault().createVector2D(5d, 1d));
    final SimplePathState y =
        SimplePathState.getPathState(
            SimpleInferredPath.getInferredPath(SimplePathEdge.getEdge(
                p1.getPathEdges().get(1).getInferredEdge(), 0d,
                false)),
            VectorFactory.getDenseDefault().createVector2D(
                5d, -1d));
    final Vector diff = y.minus(x);
    AssertJUnit.assertEquals("distance", 10d, diff.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("velocity", -2d, diff.getElement(1),
        _numericError);

    /*
     * Check for symmetry
     */
    final Vector diff2 = x.minus(y);
    AssertJUnit.assertTrue("rev diff", diff2.scale(-1).equals(diff));
  }

  @Test(enabled = false)
  public void testDistanceBetween6() {
    /*
     * FIXME!
     * Path2 is the second edge in path1, but
     * opposite direction.
     */
    final SimplePathState x =
        SimplePathState.getPathState(p1, VectorFactory
            .getDenseDefault().createVector2D(5d, 1d));
    final SimplePathState y =
        SimplePathState.getPathState(
            SimpleInferredPath.getInferredPath(SimplePathEdge.getEdge(
                p1.getPathEdges().get(1).getInferredEdge(), 0d,
                true)),
            VectorFactory.getDenseDefault().createVector2D(
                0d, -1d));
    final Vector diff = y.minus(x);
    AssertJUnit.assertEquals("distance", -15d, diff.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("velocity", 0d, diff.getElement(1),
        _numericError);

    /*
     * Check for symmetry
     */
    final Vector diff2 = x.minus(y);
    AssertJUnit.assertEquals("rev distance", -15d, diff2.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("rev velocity", 0d, diff2.getElement(1),
        _numericError);
  }

  /**
   * Deferring these non-overlapping tests until we decide to implement this.
   */
  @Test(enabled = false)
  public void testDistanceBetween7() {
    /*
     * Test that edges touching the start or end, same direction, 
     * but not overlapping, are correctly assessed.
     */

    /*
     * Touches end
     */
    final SimplePathState x =
        SimplePathState.getPathState(p1, VectorFactory
            .getDenseDefault().createVector2D(5d, 1d));
    final SimplePathState y =
        SimplePathState.getPathState(SimpleInferredPath
            .getInferredPath(SimplePathEdge.getEdge(ie3, 0d,
                false)), VectorFactory.getDenseDefault()
            .createVector2D(5d, 1d));
    final Vector diff = y.minus(x);
    AssertJUnit.assertEquals("distance", 20d, diff.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("velocity", 0d, diff.getElement(1),
        _numericError);

    /*
     * Check for symmetry, and that 
     * when something touches the beginning it works
     */
    final Vector diff2 = x.minus(y);
    AssertJUnit.assertTrue("rev diff", diff2.scale(-1).equals(diff));
  }

  /**
   * Deferring these non-overlapping tests until we decide to implement this.
   */
  @Test(enabled = false)
  public void testDistanceBetween8() {
    /*
     * Test that edges touching the start or end, but
     * not overlapping, are correctly assessed, when
     * the paths are opposite directions.
     * 
     * Again, due to the opposite directions, we
     * get the same answer in reverse.
     * (Now commutative)
     */

    /*
     * Touches end
     */
    final SimplePathState x =
        SimplePathState.getPathState(p1, VectorFactory
            .getDenseDefault().createVector2D(5d, 1d));
    final SimplePathState y =
        SimplePathState.getPathState(SimpleInferredPath
            .getInferredPath(SimplePathEdge.getEdge(ie3_rev, 0d,
                false)), VectorFactory.getDenseDefault()
            .createVector2D(5d, -1d));
    final Vector diff = y.minus(x);
    AssertJUnit.assertEquals("distance", -20d, diff.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("velocity", 0d, diff.getElement(1),
        _numericError);

    /*
     * Check for symmetry, and that 
     * when something touches the beginning it works
     */
    final Vector diff2 = x.minus(y);
    AssertJUnit.assertEquals("rev distance", -20d, diff2.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("rev velocity", 0d, diff2.getElement(1),
        _numericError);
  }

  /**
   * Deferring this test until we implement the functionality.
   */
  @Test(enabled = false)
  public void testDistanceBetween9() {
    /*
     * Test large overlaps (one edge
     * covers two small edges), 
     * and opposite directions.
     */

    final List<Coordinate> p1RevGeomCoords =
        Lists.newArrayList(p1.getGeometry().reverse()
            .getCoordinates());
    final InferredPath pOther =
        makeTmpPath(false,
            Iterables.getFirst(p1RevGeomCoords, null),
            Iterables.getLast(p1RevGeomCoords, null));
    final SimplePathState x =
        SimplePathState.getPathState(p1, VectorFactory
            .getDenseDefault().createVector2D(5d, 1d));
    final SimplePathState y =
        SimplePathState.getPathState(pOther, VectorFactory
            .getDenseDefault().createVector2D(5d, -1d));
    final Vector diff = y.minus(x);
    AssertJUnit.assertEquals("distance", -10d, diff.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("velocity", 0d, diff.getElement(1),
        _numericError);

    final Vector diff2 = x.minus(y);
    AssertJUnit.assertEquals("rev distance", -10d, diff2.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("rev velocity", 0d, diff2.getElement(1),
        _numericError);
  }

  @Test
  public void testNonSimplePathCombine1() {

    /* 
     *         +--+->  path 1 
     *         |  |  
     *  path 2 v  |
     *         ----
     */
    final InferredPath path1 =
        makeTmpPath(false, new Coordinate(0, 10),
            new Coordinate(0, 0), new Coordinate(10, 0),
            new Coordinate(10, 10), new Coordinate(20, 10));

    final InferredPath path2 =
        makeTmpPath(false, new Coordinate(20, 10),
            new Coordinate(10, 10), new Coordinate(0, 10),
            new Coordinate(0, 0));

    final Geometry from = path1.getGeometry();
    final Geometry to = path2.getGeometry();

    final PathUtils.PathMergeResults res1 =
        PathUtils.mergePaths(from, 35d, to, 35d);

    final Geometry expRes1 =
        JTSFactoryFinder.getGeometryFactory()
            .createLineString(
                new Coordinate[] { new Coordinate(0, 10),
                    new Coordinate(0, 0),
                    new Coordinate(10, 0),
                    new Coordinate(10, 10),
                    new Coordinate(20, 10),
                    new Coordinate(10, 10),
                    new Coordinate(0, 10),
                    new Coordinate(0, 0) });

    AssertJUnit.assertTrue(res1.getPath().equalsExact(expRes1));
    AssertJUnit.assertTrue(!res1.isToIsReversed());

    final PathUtils.PathMergeResults res2 =
        PathUtils.mergePaths(from, 35d,
            to.reverse(), 5d);

    AssertJUnit.assertTrue(res2.getPath().equalsExact(expRes1));
    AssertJUnit.assertTrue(res2.isToIsReversed());

    final PathUtils.PathMergeResults res3 =
        PathUtils.mergePaths(from, 2.5d, to, 5d);

    final Geometry expRes3 =
        JTSFactoryFinder.getGeometryFactory()
            .createLineString(
                new Coordinate[] { new Coordinate(0, 10),
                    new Coordinate(0, 0),
                    new Coordinate(10, 0),
                    new Coordinate(10, 10),
                    new Coordinate(20, 10), });

    AssertJUnit.assertTrue(res3.getPath().equalsExact(expRes3));
    AssertJUnit.assertTrue(res3.isToIsReversed());

    final InferredPath path11 =
        makeTmpPath(false, new Coordinate(10, 10),
            new Coordinate(20, 10), new Coordinate(20, 20));

    final PathUtils.PathMergeResults res4 =
        PathUtils.mergePaths(path11.getGeometry(),
            17.5d, to, 2.5d);

    final Geometry expRes2 =
        JTSFactoryFinder.getGeometryFactory()
            .createLineString(
                new Coordinate[] {
                    //               new Coordinate(0, 0),
                    //               new Coordinate(0, 10),
                    new Coordinate(10, 10),
                    new Coordinate(20, 10),
                    new Coordinate(20, 20) });

    AssertJUnit.assertTrue(res4.getPath().equalsExact(expRes2));
    AssertJUnit.assertTrue(res4.isToIsReversed());

    final PathUtils.PathMergeResults res5 =
        PathUtils.mergePaths(path11.getGeometry(),
            7.5d, to, 2.5d);

    final Geometry expRes5 =
        JTSFactoryFinder.getGeometryFactory()
            .createLineString(
                new Coordinate[] { new Coordinate(10, 10),
                    new Coordinate(20, 10),
                    new Coordinate(20, 20)
                //               new Coordinate(10, 10),
                //               new Coordinate(0, 10),
                //               new Coordinate(0, 0)
                });

    AssertJUnit.assertTrue(res5.getPath().equalsExact(expRes5));
    AssertJUnit.assertTrue(res5.isToIsReversed());

    final Geometry p12 =
        JTSFactoryFinder.getGeometryFactory()
            .createLineString(
                new Coordinate[] { new Coordinate(10, 10),
                    new Coordinate(20, 10),
                    new Coordinate(10, 10),
                    new Coordinate(0, 10),
                    new Coordinate(0, 0) });
    final PathUtils.PathMergeResults res6 =
        PathUtils
            .mergePaths(p12, 37.5d, p12, 32.5d);

    AssertJUnit.assertTrue(res6.getPath().equalsExact(p12));
    AssertJUnit.assertTrue(!res6.isToIsReversed());

    final PathUtils.PathMergeResults res7 =
        PathUtils.mergePaths(p12, 37.5d,
            p12.reverse(), 32.5d);

    AssertJUnit.assertTrue(res7.getPath().equalsExact(p12));
    AssertJUnit.assertTrue(res7.isToIsReversed());
  }

  @BeforeMethod
  public void testSetup() {
    /*
     * Create three stacked edges facing up the y-axis, spaced
     * by 10 meters.
     */
    graph = mock(InferenceGraph.class);
    
    Geometry e1 = JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] { new Coordinate(0, 10), new Coordinate(0, 20) });
    stub(graph.edgeHasReverse(e1)).toReturn(false);
    Geometry e2 = JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] { new Coordinate(0, 20), new Coordinate(0, 30) });
    stub(graph.edgeHasReverse(e2)).toReturn(false);
    Geometry e3 = JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] { new Coordinate(0, 30), new Coordinate(0, 40) });
    stub(graph.edgeHasReverse(e3)).toReturn(false);
    ie1 = SimpleInferredEdge.getInferredEdge(e1, null, 1, graph);
    ie2 = SimpleInferredEdge.getInferredEdge(e2, null, 2, graph);
    ie3 = SimpleInferredEdge.getInferredEdge(e3, null, 3, graph);
    ie1_rev = SimpleInferredEdge.getInferredEdge(e1.reverse(), null, -1, graph);
    ie2_rev = SimpleInferredEdge.getInferredEdge(e2.reverse(), null, -2, graph);
    ie3_rev = SimpleInferredEdge.getInferredEdge(e3.reverse(), null, -3, graph);
    
    
      p1 = makeTmpPath(false, new Coordinate(0, 10),
            new Coordinate(0, 20), new Coordinate(0, 30));
      
      p2 = makeTmpPath(false, new Coordinate(0, 20),
            new Coordinate(0, 30), new Coordinate(0, 40));
      
      p1_neg = makeTmpPath(true, new Coordinate(0, 10),
            new Coordinate(0, 20), new Coordinate(0, 30));
      
      p2_neg = makeTmpPath(true, new Coordinate(0, 20),
            new Coordinate(0, 30), new Coordinate(0, 40));
      
      p2_rev = makeTmpPath(false, new Coordinate(0, 40),
            new Coordinate(0, 30), new Coordinate(0, 20));

  }
}
