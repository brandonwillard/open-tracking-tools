package org.opentrackingtools.paths;

import static org.mockito.Mockito.mock;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.Date;

import org.opentrackingtools.VehicleStateInitialParameters;
import org.opentrackingtools.distributions.AdjMultivariateGaussian;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.estimators.PathStateEstimatorPredictor;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.ProjectedCoordinate;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.util.PathUtils;
import org.opentrackingtools.util.SvdMatrix;
import org.opentrackingtools.util.TestUtils;
import org.testng.AssertJUnit;
import org.testng.annotations.BeforeMethod;
import org.testng.annotations.Test;

import com.google.common.collect.Iterables;
import com.vividsolutions.jts.geom.Coordinate;

public class PathTest {

  private VehicleStateInitialParameters vehicleStateInitialParams;
  private InferenceGraph graph;

  @BeforeMethod
  public void setUp() throws Exception {

    vehicleStateInitialParams =
        new VehicleStateInitialParameters(
            null,
            VectorFactory.getDefault().createVector2D(100d, 100d), 20,
            VectorFactory.getDefault().createVector1D(
                0.000625), 20, VectorFactory.getDefault()
                .createVector2D(0.000625, 0.000625), 20,
            VectorFactory.getDefault().createVector2D(5d,
                95d), VectorFactory.getDefault()
                .createVector2D(95d, 5d),
            25, 30, 0l);

    graph = mock(InferenceGraph.class);
  }

  @Test(enabled=false, expectedExceptions = IllegalStateException.class)
  public void testBadConversion() {
    final Path startPath =
        TestUtils.makeTmpPath(graph, true,
            new Coordinate(20, -10),
            new Coordinate(10, -10));

    final Matrix covar =
        MatrixFactory.getDefault().copyArray(
            new double[][] { new double[] { 126.56, 8.44 },
                new double[] { 8.44, 0.56 } });
    final MultivariateGaussian startBelief =
        new AdjMultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(-0d, -5d / 30d), new SvdMatrix(covar));
    final PathStateDistribution currentBelief =
        new PathStateDistribution(startPath,
            startBelief);

    final Path newPath =
        TestUtils.makeTmpPath(graph, true,
            new Coordinate(20, -10),
            new Coordinate(10, -10), new Coordinate(10, 0),
            new Coordinate(0, 0));

    currentBelief.convertToPath(newPath);
  }

  /**
   * Test conversion from a positive state to a negative one with an opposite
   * geom.
   */
  @Test
  public void testGetStateOnPath() {

    final Path startPath =
        TestUtils.makeTmpPath(graph, false,
            new Coordinate(0, 0), new Coordinate(0, 60));

    final PathState startState =
        new PathState(startPath, VectorFactory
            .getDefault().createVector2D(12d, -1));

    final Path newPath =
        TestUtils.makeTmpPath(graph, true,
            new Coordinate(0, 60), new Coordinate(0, 0));

    final Vector result = startState.convertToPath(newPath);

    AssertJUnit.assertEquals("dist", -12d, result.getElement(0), 0d);
    AssertJUnit.assertEquals("dist", 1d, result.getElement(1), 0d);

  }

  /**
   * Test around the start of the edge, positive.
   */
  @Test
  public void testPathEdge1() {
    final Path path1 =
        TestUtils.makeTmpPath(graph, false,
            new Coordinate(0, 10.11d), new Coordinate(
                10.11d, 20d));

    final PathEdge edge1 =
        Iterables.getFirst(path1.getPathEdges(), null);

    final Vector testVec1 =
        VectorFactory.getDenseDefault().createVector2D(
            -1e-12d, -1d);
    final Vector result1 =
        edge1.getCheckedStateOnEdge(testVec1, 1e-7d, false);

    AssertJUnit.assertEquals(0d, result1.getElement(0), 0d);

    final Vector testVec2 =
        VectorFactory.getDenseDefault().createVector2D(
            1e-12d, -1d);
    final Vector result2 =
        edge1.getCheckedStateOnEdge(testVec2, 1e-7d, false);

    AssertJUnit.assertEquals(1e-12d, result2.getElement(0), 0d);

    final Vector testVec3 =
        VectorFactory.getDenseDefault().createVector2D(
            -1e-6d, -1d);
    final Vector result3 =
        edge1.getCheckedStateOnEdge(testVec3, 1e-7d, false);

    AssertJUnit.assertEquals(null, result3);

  }

  /**
   * Test around the start of the edge, negative.
   */
  @Test
  public void testPathEdge1_2() {
    final Path path1 =
        TestUtils.makeTmpPath(graph, true,
            new Coordinate(0, 20d), new Coordinate(
                1d / 10d, 10.11d));

    final PathEdge edge1 =
        Iterables.getFirst(path1.getPathEdges(), null);

    final Vector testVec1 =
        VectorFactory.getDenseDefault().createVector2D(
            1e-12d, -1d);
    final Vector result1 =
        edge1.getCheckedStateOnEdge(testVec1, 1e-7d, false);

    AssertJUnit.assertEquals(0d, result1.getElement(0), 0d);

    final Vector testVec2 =
        VectorFactory.getDenseDefault().createVector2D(
            -1e-12d, -1d);
    final Vector result2 =
        edge1.getCheckedStateOnEdge(testVec2, 1e-7d, false);

    AssertJUnit.assertEquals(-1e-12d, result2.getElement(0), 0d);

    final Vector testVec3 =
        VectorFactory.getDenseDefault().createVector2D(
            1e-6d, -1d);
    final Vector result3 =
        edge1.getCheckedStateOnEdge(testVec3, 1e-7d, false);

    AssertJUnit.assertEquals(null, result3);

  }

  @Test
  public void testPathStateConvert1() {
    final Path startPath =
        TestUtils.makeTmpPath(graph, true,
            new Coordinate(0, 0), new Coordinate(10, 0));

    final Matrix covar =
        MatrixFactory.getDefault().copyArray(
            new double[][] { new double[] { 126.56, 8.44 },
                new double[] { 8.44, 0.56 } });
    final MultivariateGaussian startBelief =
        new AdjMultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(-0d, -5d / 30d), new SvdMatrix(covar));
    final PathStateDistribution currentBelief =
        new PathStateDistribution(startPath,
            startBelief);

    final Vector groundLoc =
        MotionStateEstimatorPredictor.getOg().times(
            currentBelief.getGroundDistribution().getMean());
    AssertJUnit.assertEquals("initial state x", 10d,
        groundLoc.getElement(0), 0d);
    AssertJUnit.assertEquals("initial state y", 0d,
        groundLoc.getElement(1), 0d);

    final Path newPath =
        TestUtils.makeTmpPath(graph, true,
            new Coordinate(20, -10),
            new Coordinate(10, -10), new Coordinate(10, 0),
            new Coordinate(0, 0));

    final PathStateDistribution result = currentBelief.convertToPath(newPath);

    AssertJUnit.assertEquals("distance", -10d, result
        .getMotionDistribution().getMean().getElement(0), 0d);
    AssertJUnit.assertEquals("velocity", 5d / 30d, result
        .getMotionDistribution().getMean().getElement(1), 0d);

  }

  @Test
  public void testPathStateConvert2() {
    final Path startPath =
        TestUtils.makeTmpPath(graph, false,
            new Coordinate(0, 0), new Coordinate(10, 0));

    final Matrix covar =
        MatrixFactory.getDefault().copyArray(
            new double[][] { new double[] { 126.56, 8.44 },
                new double[] { 8.44, 0.56 } });
    final MultivariateGaussian startBelief =
        new AdjMultivariateGaussian(VectorFactory.getDenseDefault()
            .createVector2D(0d, 1d), new SvdMatrix(covar));
    final PathStateDistribution currentBelief =
        new PathStateDistribution(startPath,
            startBelief);

    final Vector groundLoc =
        MotionStateEstimatorPredictor.getOg().times(
            currentBelief.getGroundDistribution().getMean());
    AssertJUnit.assertEquals("initial state x", 0d,
        groundLoc.getElement(0), 0d);
    AssertJUnit.assertEquals("initial state y", 0d,
        groundLoc.getElement(1), 0d);

    final Path newPath =
        TestUtils.makeTmpPath(graph, true,
            new Coordinate(20, -10),
            new Coordinate(10, -10), new Coordinate(10, 0),
            new Coordinate(0, 0));

    final PathStateDistribution result = currentBelief.convertToPath(newPath);

    AssertJUnit.assertEquals("distance", -0d, result
        .getMean().getElement(0), 0d);
    AssertJUnit.assertEquals("velocity", -1d, result
        .getMean().getElement(1), 0d);

  }

  @Test
  public void testPathStateConvert3() {
    final Path startPath =
        TestUtils.makeTmpPath(graph, false,
            new Coordinate(0, 0), new Coordinate(10, 0));

    final Matrix covar =
        MatrixFactory.getDefault().copyArray(
            new double[][] { new double[] { 126.56, 8.44 },
                new double[] { 8.44, 0.56 } });
    final MultivariateGaussian startBelief =
        new AdjMultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(2.5d, 1d), new SvdMatrix(covar));
    final PathStateDistribution currentBelief =
        new PathStateDistribution(startPath,
            startBelief);

    final Vector groundLoc =
        MotionStateEstimatorPredictor.getOg().times(
            currentBelief.getGroundDistribution().getMean());
    AssertJUnit.assertEquals("initial state x", 2.5d,
        groundLoc.getElement(0), 0d);
    AssertJUnit.assertEquals("initial state y", 0d,
        groundLoc.getElement(1), 0d);

    final Path newPath =
        TestUtils.makeTmpPath(graph, false,
            new Coordinate(0, 0), new Coordinate(10, 0),
            new Coordinate(10, -10),
            new Coordinate(20, -10));

    final PathStateDistribution result = currentBelief.convertToPath(newPath);

    AssertJUnit.assertEquals("distance", 2.5d, result
        .getMean().getElement(0), 0d);
    AssertJUnit.assertEquals("velocity", 1d, result
        .getMean().getElement(1), 0d);

  }

  @Test
  public void testPathStateConvert4() {
    final Path startPath =
        TestUtils.makeTmpPath(graph, true,
            new Coordinate(10, 0), new Coordinate(0, 0));

    final Matrix covar =
        MatrixFactory.getDefault().copyArray(
            new double[][] { new double[] { 126.56, 8.44 },
                new double[] { 8.44, 0.56 } });
    final MultivariateGaussian startBelief =
        new AdjMultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(-2.5d, 1d), new SvdMatrix(covar));
    final PathStateDistribution currentBelief =
        new PathStateDistribution(startPath,
            startBelief);

    final Path newPath =
        TestUtils.makeTmpPath(graph, true,
            new Coordinate(20, -10),
            new Coordinate(10, -10), new Coordinate(10, 0),
            new Coordinate(0, 0));

    final PathStateDistribution result = currentBelief.convertToPath(newPath);

    AssertJUnit.assertEquals("distance", -2.5d, result
        .getMean().getElement(0), 0d);
    AssertJUnit.assertEquals("velocity", 1d, result
        .getMean().getElement(1), 0d);

  }

  /**
   * Path edges are reverse, directions are the same.
   */
  @Test
  public void testPathStateConvert5() {

    final Path startPath =
        TestUtils.makeTmpPath(graph, false,
            new Coordinate(0, 0), new Coordinate(10, 0),
            new Coordinate(10, -10),
            new Coordinate(20, -10));
    final Matrix covar =
        MatrixFactory.getDefault().copyArray(
            new double[][] { new double[] { 126.56, 8.44 },
                new double[] { 8.44, 0.56 } });
    final MultivariateGaussian startBelief =
        new AdjMultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(2.5d, 1d), new SvdMatrix(covar));
    final PathStateDistribution currentBelief =
        new PathStateDistribution(startPath,
            startBelief);

    final Path newPath =
        TestUtils.makeTmpPath(graph, false,
            new Coordinate(10, 0), new Coordinate(0, 0));

    final PathStateDistribution result = currentBelief.convertToPath(newPath);

    AssertJUnit.assertEquals("distance", 7.5d, result
        .getMean().getElement(0), 0d);
    AssertJUnit.assertEquals("velocity", -1d, result
        .getMean().getElement(1), 0d);

  }

  /**
   * Path & edge are the same, directions are reverse, pos. to neg.
   */
  @Test
  public void testPathStateConvert6() {

    final Path startPath =
        TestUtils.makeTmpPath(graph, false,
            new Coordinate(0, 0), new Coordinate(10, 0),
            new Coordinate(10, -10),
            new Coordinate(20, -10));
    final Matrix covar =
        MatrixFactory.getDefault().copyArray(
            new double[][] { new double[] { 126.56, 8.44 },
                new double[] { 8.44, 0.56 } });
    final MultivariateGaussian startBelief =
        new AdjMultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(2.5d, 1d), new SvdMatrix(covar));
    final PathStateDistribution currentBelief =
        new PathStateDistribution(startPath,
            startBelief);

    final Path newPath =
        TestUtils.makeTmpPath(graph, true,
            new Coordinate(0, 0), new Coordinate(10, 0));

    final PathStateDistribution result = currentBelief.convertToPath(newPath);

    AssertJUnit.assertEquals("distance", -7.5d, result
        .getMean().getElement(0), 0d);
    AssertJUnit.assertEquals("velocity", 1d, result
        .getMean().getElement(1), 0d);

  }

  /**
   * Path & edge are the same, directions are reverse, neg. to pos.
   */
  @Test
  public void testPathStateConvert7() {

    final Path startPath =
        TestUtils.makeTmpPath(graph, true,
            new Coordinate(20, -10),
            new Coordinate(10, -10), new Coordinate(10, 0),
            new Coordinate(0, 0));
    final Matrix covar =
        MatrixFactory.getDefault().copyArray(
            new double[][] { new double[] { 126.56, 8.44 },
                new double[] { 8.44, 0.56 } });
    final MultivariateGaussian startBelief =
        new AdjMultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(-2.5d, 1d), new SvdMatrix(covar));
    final PathStateDistribution currentBelief =
        new PathStateDistribution(startPath,
            startBelief);

    final Path newPath =
        TestUtils.makeTmpPath(graph, false,
            new Coordinate(10, 0), new Coordinate(0, 0));

    final PathStateDistribution result = currentBelief.convertToPath(newPath);

    AssertJUnit.assertEquals("distance", 7.5d, result
        .getMean().getElement(0), 0d);
    AssertJUnit.assertEquals("velocity", 1d, result
        .getMean().getElement(1), 0d);

  }

}
