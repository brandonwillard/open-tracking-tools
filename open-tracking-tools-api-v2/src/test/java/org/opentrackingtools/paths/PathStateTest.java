package org.opentrackingtools.paths;

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
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.util.PathUtils;
import org.opentrackingtools.util.TestUtils;

import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineSegment;

public class PathStateTest {

  private final double _numericError = 1e-4;
  
  private InferenceGraph graph = mock(InferenceGraph.class);

  private Path makeTmpPath(boolean isBackward,
    Coordinate... coords) {
    return TestUtils.makeTmpPath(this.graph,
        isBackward, coords);
  }

  @Test(expectedExceptions = IllegalArgumentException.class)
  public void testBadPathError() {
    
    Geometry e2 = JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] { new Coordinate(0, 20), new Coordinate(0, 30) });
    stub(graph.edgeHasReverse(e2)).toReturn(false);
    final InferenceGraphEdge ie2_rev = new InferenceGraphEdge(e2.reverse(), e2.reverse(), -2, graph);
    
    Geometry e3 = JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] { new Coordinate(0, 30), new Coordinate(0, 40) });
    stub(graph.edgeHasReverse(e3)).toReturn(false);
    final InferenceGraphEdge ie3_rev = new InferenceGraphEdge(e3.reverse(), e3.reverse(), -3, graph);
    
    final List<PathEdge> edges2_rev =
        Lists.newArrayList(
            new PathEdge(Iterables.getOnlyElement(ie2_rev.getSegments()), 0d, false), 
            new PathEdge(Iterables.getOnlyElement(ie3_rev.getSegments()), ie2_rev.getLength(), false));

    new Path(edges2_rev, false);

  }

  @Test
  public void testDistanceBetween1() {
    /*
     * States on the same path.
     */
    Path p1 = makeTmpPath(false, new Coordinate(0, 10),
          new Coordinate(0, 20), new Coordinate(0, 30));
    final PathState x =
        new PathState(p1, VectorFactory
            .getDenseDefault().createVector2D(5d, 1d));
    final PathState y =
        new PathState(p1, VectorFactory
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

    final Path path1 =
        makeTmpPath(false, new Coordinate(0, 10),
            new Coordinate(0, 0), new Coordinate(10, 0),
            new Coordinate(10, 10), new Coordinate(20, 10));

    final Path path2 =
        makeTmpPath(false, new Coordinate(20, 10),
            new Coordinate(10, 10), new Coordinate(0, 10),
            new Coordinate(0, 0));

    final PathState x =
        new PathState(path1, VectorFactory
            .getDenseDefault().createVector2D(35d, 1d));
    final PathState y =
        new PathState(path2, VectorFactory
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

    final PathState x2 =
        new PathState(path1, VectorFactory
            .getDenseDefault().createVector2D(22.5d, 1d));
    final PathState y2 =
        new PathState(path2, VectorFactory
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

    final Path path1 =
        makeTmpPath(false, new Coordinate(0, 0),
            new Coordinate(0, 10));

    final Path path2 =
        makeTmpPath(false, new Coordinate(0, 10),
            new Coordinate(0, 0), new Coordinate(-10, 0));

    final PathState x =
        new PathState(path2, VectorFactory
            .getDenseDefault().createVector2D(15d, 1d));
    final PathState y =
        new PathState(path1, VectorFactory
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
    final Path startPath =
        makeTmpPath(false, new Coordinate(0, 0),
            new Coordinate(0, 60));

    final PathState startState =
        new PathState(startPath, VectorFactory
            .getDefault().createVector2D(12d, -1));

    final Path newPath =
        makeTmpPath(true, new Coordinate(0, 60),
            new Coordinate(0, 0));

    final PathState result =
        startState.convertToPath(newPath);

    final PathState stateOnNewPath =
        new PathState(newPath, result.getMotionState());
    final Vector difference =
        stateOnNewPath.minus(startState);

    AssertJUnit.assertTrue(difference.isZero(1e-7));
  }

  @Test
  public void testDistanceBetween13() {
    final Path startPath =
        makeTmpPath(false, new Coordinate(0, 0),
            new Coordinate(0, 60), new Coordinate(0, 199));

    final PathState startState =
        new PathState(startPath, VectorFactory
            .getDefault().createVector2D(104d, -1));

    final Path newPath =
        makeTmpPath(false, new Coordinate(0, 199),
            new Coordinate(0, 60));

    final PathState result =
        startState.convertToPath(newPath);

    final PathState stateOnNewPath =
        new PathState(newPath, result.globalState);
    final Vector difference =
        stateOnNewPath.minus(startState);

    AssertJUnit.assertTrue(difference.isZero(1e-7));
  }

  @Test
  public void testDistanceBetween14() {
    final Path startPath =
        makeTmpPath(true, new Coordinate(0, 199),
            new Coordinate(0, 60), new Coordinate(0, 0));

    final PathState startState =
        new PathState(startPath, VectorFactory
            .getDefault().createVector2D(-104d, -2.79));

    final Path newPath =
        makeTmpPath(true, new Coordinate(0, 60),
            new Coordinate(0, 199));

    final PathState result =
        startState.convertToPath(newPath);

    final PathState stateOnNewPath =
        new PathState(newPath, result.globalState);
    final Vector difference =
        stateOnNewPath.minus(startState);

    AssertJUnit.assertTrue(difference.isZero(1e-7));
  }

  @Test
  public void testDistanceBetween15() {
    final Path startPath =
        makeTmpPath(true, new Coordinate(0,
            178.89095954422947), new Coordinate(0, 0),
            new Coordinate(0, 139.9919743536118d));

    final PathState startState =
        new PathState(
            startPath,
            VectorFactory.getDefault().createVector2D(
                -139.9919743536118d, -3.39));

    final Path newPath =
        makeTmpPath(false, new Coordinate(0, 0),
            new Coordinate(0, 178.89095954422947));

    final PathState result =
        startState.convertToPath(newPath);

    final PathState stateOnNewPath =
        new PathState(newPath, result.globalState);
    final Vector difference =
        stateOnNewPath.minus(startState);

    AssertJUnit.assertTrue(difference.isZero(1e-7));
  }

  @Test
  public void testDistanceBetween16() {
    final Path otherPath =
        makeTmpPath(false, new Coordinate(0d, 0d),
            new Coordinate(-85d, 0d), new Coordinate(
                -150.37d, 0d));

    final PathState otherState =
        new PathState(otherPath, VectorFactory
            .getDefault().createVector2D(121.15d, 2.59d));

    final Path thisPath =
        makeTmpPath(true, new Coordinate(0d, 0d),
            new Coordinate(-85d, 0d), new Coordinate(
                -150.37d, 0d));
    final PathState thisState =
        new PathState(thisPath, VectorFactory.getDefault().createVector2D(
                    -150.37d, -12.06d));

    final Vector difference = thisState.minus(otherState);

    AssertJUnit.assertEquals("dist", -121d, difference.getElement(0),
        1d);
    AssertJUnit.assertEquals("vel", -14.65d, difference.getElement(1),
        1d);
  }

  @Test
  public void testDistanceBetween17() {
    final Path startPath =
        makeTmpPath(true, new Coordinate(0, -262.21d),
            new Coordinate(0, 0),
            new Coordinate(0, 792.85d));

    final PathState startState =
        new PathState(
            startPath,
            VectorFactory.getDefault().createVector2D(
                -808.4862233549034, -26.2880263830998));

    final Path newPath =
        makeTmpPath(true, new Coordinate(0, 792.85d),
            new Coordinate(0, 0), new Coordinate(0,
                -262.21d));

    final Vector result =
        startState.convertToPath(newPath).getMotionState();

    final PathState stateOnNewPath =
        new PathState(newPath, result);
    final Vector difference =
        stateOnNewPath.minus(startState);

    AssertJUnit.assertTrue(difference.isZero(1e-7));
  }

  @Test
  public void testDistanceBetween18() {
    final Path otherPath =
        makeTmpPath(true, new Coordinate(-90d, 0d),
            new Coordinate(0d, 0d), new Coordinate(0d,
                -(517d - 90d) / 2d),
            new Coordinate(0d, 0d),
            new Coordinate(-90d, 0d));

    final PathState otherState =
        new PathState(otherPath, VectorFactory
            .getDefault().createVector2D(-564d, -8d));

    final Path thisPath =
        makeTmpPath(false, new Coordinate(0d, 0d),
            new Coordinate(-90d, 0d), new Coordinate(-90d,
                607d - 90d));
    final PathState thisState =
        new PathState(thisPath, VectorFactory
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
    final Path otherPath =
        makeTmpPath(true, new Coordinate(-90d, 0d),
            new Coordinate(0d, 0d), new Coordinate(0d,
                -(517d - 90d) / 2d),
            new Coordinate(0d, 0d),
            new Coordinate(-90d, 0d));

    final PathState otherState =
        new PathState(otherPath, VectorFactory
            .getDefault().createVector2D(-564d, -8d));

    /*
     * This path starts at the same start as
     * the other. 
     */
    final Path thisPath =
        makeTmpPath(true, new Coordinate(0d, 607d - 90d),
            new Coordinate(0d, 0d),
            new Coordinate(-90d, 0d));

    final Vector result =
        otherState.convertToPath(thisPath).getMotionState();

    final PathState stateOnThis =
        new PathState(thisPath, result);
    final Vector difference = stateOnThis.minus(otherState);

    AssertJUnit.assertTrue(difference.isZero(1e-7));
  }

  @Test
  public void testDistanceBetween2() {
    Path p1 = makeTmpPath(false, new Coordinate(0, 10),
          new Coordinate(0, 20), new Coordinate(0, 30));
    final PathState x =
        new PathState(p1, VectorFactory
            .getDenseDefault().createVector2D(5d, 1d));
    final Path p1_neg = makeTmpPath(true, new Coordinate(0, 10),
          new Coordinate(0, 20), new Coordinate(0, 30));
    final PathState y =
        new PathState(
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
    final Path otherPath =
        makeTmpPath(true, new Coordinate(0d, 0d),
            new Coordinate(0d, 179d), new Coordinate(10d,
                179d), new Coordinate(56d, 179d));

    final PathState otherState =
        new PathState(otherPath, VectorFactory
            .getDefault().createVector2D(-41d, 4d));

    final Path thisPath =
        makeTmpPath(false, new Coordinate(0d, 0d),
            new Coordinate(0d, 179d));
    final PathState thisState =
        new PathState(thisPath, VectorFactory
            .getDefault().createVector2D(157d, 7.5d));

    final Vector difference = thisState.minus(otherState);

    AssertJUnit.assertEquals("dist", -37d, difference.getElement(0), 0d);
    AssertJUnit.assertEquals("vel", 3.5d, difference.getElement(1), 0d);
  }

  @Test
  public void testDistanceBetween21() {
    final Path otherPath =
        makeTmpPath(false, new Coordinate(0d, 0d),
            new Coordinate(100d, 0d));

    final PathState otherState =
        new PathState(otherPath, VectorFactory
            .getDefault().createVector2D(49d, 1.6d));

    final Path thisPath =
        makeTmpPath(false, new Coordinate(0d, 0d),
            new Coordinate(100d, 0d), new Coordinate(100d,
                100d), new Coordinate(100d, 0d),
            new Coordinate(0d, 0d));
    final PathState thisState =
        new PathState(thisPath, VectorFactory
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
    final Path otherPath =
        makeTmpPath(false, new Coordinate(0d, 0d),
            new Coordinate(100d, 0d), new Coordinate(100d,
                100d), new Coordinate(100d, 0d),
            new Coordinate(0d, 0d));

    final PathState otherState =
        new PathState(otherPath, VectorFactory
            .getDefault().createVector2D(49d, 1.6d));

    final Path thisPath =
        makeTmpPath(false, new Coordinate(0d, 0d),
            new Coordinate(100d, 0d));
    final PathState thisState =
        new PathState(thisPath, VectorFactory
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
    Path p1 = makeTmpPath(false, new Coordinate(0, 10),
          new Coordinate(0, 20), new Coordinate(0, 30));
    final PathState x =
        new PathState(p1, VectorFactory
            .getDenseDefault().createVector2D(5d, 1d));
    final Path p1_neg = makeTmpPath(true, new Coordinate(0, 10),
          new Coordinate(0, 20), new Coordinate(0, 30));
    final PathState y =
        new PathState(
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
    Path p1 = makeTmpPath(false, new Coordinate(0, 10),
          new Coordinate(0, 20), new Coordinate(0, 30));
    final PathState x =
        new PathState(p1, VectorFactory
            .getDenseDefault().createVector2D(12.5d, 1d));
    final Path p2 = makeTmpPath(false, new Coordinate(0, 20),
          new Coordinate(0, 30), new Coordinate(0, 40));
    final PathState y =
        new PathState(p2, VectorFactory
            .getDenseDefault().createVector2D(17.5d, -1d));
    final Vector diff = y.minus(x);
    AssertJUnit.assertEquals("distance", 15d, diff.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("velocity", -2d, diff.getElement(1),
        _numericError);
  }

  @Test
  public void testDistanceBetween4() {
    
    Path p1 = makeTmpPath(false, new Coordinate(0, 10),
          new Coordinate(0, 20), new Coordinate(0, 30));
    final PathState x =
        new PathState(p1, VectorFactory
            .getDenseDefault().createVector2D(12.5d, 1d));
    final PathState y =
        new PathState(TestUtils
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
    final PathState x2 =
        new PathState(TestUtils
            .makeTmpPath(this.graph, false,
                new Coordinate(0, 30),
                new Coordinate(0, 20),
                new Coordinate(0, 10)), VectorFactory
            .getDenseDefault().createVector2D(12.5d, 1d));
    final Path p2 = makeTmpPath(false, new Coordinate(0, 20),
          new Coordinate(0, 30), new Coordinate(0, 40));
    final PathState y2 =
        new PathState(p2, VectorFactory
            .getDenseDefault().createVector2D(17.5d, 1d));
    final Vector diff2 = y2.minus(x2);
    AssertJUnit.assertEquals("distance", 20d, diff2.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("velocity", 2d, diff2.getElement(1),
        _numericError);
  }

  @Test
  public void testDistanceBetween4_3() {
    final PathState x2 =
        new PathState(TestUtils
            .makeTmpPath(this.graph, false,
                new Coordinate(0, 30),
                new Coordinate(0, 20),
                new Coordinate(0, 10)), VectorFactory
            .getDenseDefault().createVector2D(12.5d, 1d));
    final PathState y2 =
        new PathState(TestUtils
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
    Path p1 = makeTmpPath(false, new Coordinate(0, 10),
          new Coordinate(0, 20), new Coordinate(0, 30));
    final PathState x =
        new PathState(p1, VectorFactory
            .getDenseDefault().createVector2D(5d, 1d));
    final PathState y =
        new PathState(
            new Path(new PathEdge(p1.getPathEdges().get(1).getSegment(), 0d, false)),
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
    
    Path p1 = makeTmpPath(false, new Coordinate(0, 10),
          new Coordinate(0, 20), new Coordinate(0, 30));
    final PathState x =
        new PathState(p1, VectorFactory
            .getDenseDefault().createVector2D(5d, 1d));
    final PathState y =
        new PathState(
            new Path(new PathEdge(p1.getPathEdges().get(1).getSegment(),
                0d, true)),
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
    Geometry e3 = JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] { new Coordinate(0, 30), new Coordinate(0, 40) });
    stub(graph.edgeHasReverse(e3)).toReturn(false);
    final InferenceGraphEdge ie3 = new InferenceGraphEdge(e3, e3, 3, graph);
    
    Path p1 = makeTmpPath(false, new Coordinate(0, 10),
          new Coordinate(0, 20), new Coordinate(0, 30));

    /*
     * Touches end
     */
    final PathState x =
        new PathState(p1, VectorFactory
            .getDenseDefault().createVector2D(5d, 1d));
    final PathState y =
        new PathState(new Path(new PathEdge(Iterables.getOnlyElement(ie3.getSegments())
            , 0d, false)), 
            VectorFactory.getDenseDefault().createVector2D(5d, 1d));
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
    Path p1 = makeTmpPath(false, new Coordinate(0, 10),
          new Coordinate(0, 20), new Coordinate(0, 30));

    /*
     * Touches end
     */
    final PathState x =
        new PathState(p1, VectorFactory
            .getDenseDefault().createVector2D(5d, 1d));
    Geometry e3 = JTSFactoryFinder.getGeometryFactory().createLineString(
        new Coordinate[] { new Coordinate(0, 30), new Coordinate(0, 40) });
    stub(graph.edgeHasReverse(e3)).toReturn(false);
    final InferenceGraphEdge ie3_rev = new InferenceGraphEdge(e3.reverse(), e3.reverse(), -3, graph);
    final LineSegment segment = new LineSegment(ie3_rev.getGeometry().getCoordinates()[0], 
        ie3_rev.getGeometry().getCoordinates()[1]);
    final PathState y =
        new PathState(new Path(new PathEdge(Iterables.getOnlyElement(ie3_rev.getSegments()), 0d, false)), 
            VectorFactory.getDenseDefault().createVector2D(5d, -1d));
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

  @Test
  public void testDistanceBetween9() {

    Path toPath = makeTmpPath(false, 
        new Coordinate(586311.9494330664, 4512151.757246521),
        new Coordinate(586313.1113431666, 4512052.037593625), 
        new Coordinate(586311.9494330664, 4512151.757246521),
        new Coordinate(586313.1113431666, 4512052.037593625));
    Path fromPath = toPath;
    
    final PathState toState =
        new PathState(toPath, VectorFactory
            .getDenseDefault().createVector2D(203.96780646837124, 8.65530011125422));
    final PathState fromState =
        new PathState(fromPath, VectorFactory
            .getDenseDefault().createVector2D(203.96780646837124, 8.65530011125422));
    final Vector diff = toState.minus(fromState);
    AssertJUnit.assertEquals("distance", 0d, diff.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("velocity", 0d, diff.getElement(1),
        _numericError);

    final Vector diff2 = fromState.minus(toState);
    AssertJUnit.assertEquals("rev distance", 0d, diff2.getElement(0),
        _numericError);
    AssertJUnit.assertEquals("rev velocity", 0d, diff2.getElement(1),
        _numericError);
  }

  @Test
  public void testNonPathCombine1() {

    /* 
     *         +--+->  path 1 
     *         |  |  
     *  path 2 v  |
     *         ----
     */
    final Path path1 =
        makeTmpPath(false, new Coordinate(0, 10),
            new Coordinate(0, 0), new Coordinate(10, 0),
            new Coordinate(10, 10), new Coordinate(20, 10));

    final Path path2 =
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

    final Path path11 =
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
}
