package org.opentrackingtools.graph.paths.impl;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.Date;

import org.junit.Before;
import org.junit.Test;
import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.otp.impl.OtpGraph;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.edges.impl.SimplePathEdge;
import org.opentrackingtools.graph.paths.impl.SimpleInferredPath;
import org.opentrackingtools.graph.paths.states.PathStateBelief;
import org.opentrackingtools.graph.paths.states.impl.SimplePathState;
import org.opentrackingtools.graph.paths.states.impl.SimplePathStateBelief;
import org.opentrackingtools.impl.SimpleObservation;
import org.opentrackingtools.impl.TimeOrderException;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.impl.VehicleTrackingPLFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.StandardRoadTrackingFilter;
import org.opentrackingtools.util.GeoUtils;
import org.opentrackingtools.util.geom.ProjectedCoordinate;

import com.google.common.collect.Iterables;
import com.vividsolutions.jts.geom.Coordinate;

public class InferredPathTest {

  private VehicleStateInitialParameters vehicleStateInitialParams;
  private AbstractRoadTrackingFilter<?> filter;
  private OtpGraph graph;

  @Before
  public void setUp() throws Exception {

    vehicleStateInitialParams =
        new VehicleStateInitialParameters(VectorFactory
            .getDefault().createVector2D(100d, 100d), 20,
            VectorFactory.getDefault().createVector1D(
                0.000625), 20, VectorFactory.getDefault()
                .createVector2D(0.000625, 0.000625), 20,
            VectorFactory.getDefault().createVector2D(5d,
                95d), VectorFactory.getDefault()
                .createVector2D(95d, 5d),
            VehicleTrackingPLFilter.class.getName(), 25,
            30, 0l);

    filter =
        new StandardRoadTrackingFilter(
            vehicleStateInitialParams.getObsCov(),
            vehicleStateInitialParams.getOffRoadStateCov(),
            vehicleStateInitialParams.getOnRoadStateCov(),
            vehicleStateInitialParams.getInitialObsFreq());

    graph = mock(OtpGraph.class);
  }

  @Test(expected = IllegalStateException.class)
  public void testBadConversion() {
    final SimpleInferredPath startPath =
        TrackingTestUtils.makeTmpPath(graph, true,
            new Coordinate(20, -10),
            new Coordinate(10, -10));

    final Matrix covar =
        MatrixFactory.getDefault().copyArray(
            new double[][] { new double[] { 126.56, 8.44 },
                new double[] { 8.44, 0.56 } });
    final MultivariateGaussian startBelief =
        new MultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(-0d, -5d / 30d), covar);
    final SimplePathStateBelief currentBelief =
        SimplePathStateBelief.getPathStateBelief(startPath,
            startBelief);

    final InferredPath newPath =
        TrackingTestUtils.makeTmpPath(graph, true,
            new Coordinate(20, -10),
            new Coordinate(10, -10), new Coordinate(10, 0),
            new Coordinate(0, 0));

    newPath.getStateBeliefOnPath(currentBelief);

  }

  /**
   * Test conversion from a positive state to a negative one with an opposite
   * geom.
   */
  @Test
  public void testGetStateOnPath() {

    final SimpleInferredPath startPath =
        TrackingTestUtils.makeTmpPath(graph, false,
            new Coordinate(0, 0), new Coordinate(0, 60));

    final SimplePathState startState =
        SimplePathState.getPathState(startPath, VectorFactory
            .getDefault().createVector2D(12d, -1));

    final InferredPath newPath =
        TrackingTestUtils.makeTmpPath(graph, true,
            new Coordinate(0, 60), new Coordinate(0, 0));

    final Vector result =
        newPath.getStateOnPath(startState).getRawState();

    assertEquals("dist", -12d, result.getElement(0), 0d);
    assertEquals("dist", 1d, result.getElement(1), 0d);

  }

  /**
   * Test around the start of the edge, positive.
   */
  @Test
  public void testPathEdge1() {
    final InferredPath path1 =
        TrackingTestUtils.makeTmpPath(graph, false,
            new Coordinate(0, 10.11d), new Coordinate(
                10.11d, 20d));

    final PathEdge edge1 =
        Iterables.getFirst(path1.getPathEdges(), null);

    final Vector testVec1 =
        VectorFactory.getDenseDefault().createVector2D(
            -1e-12d, -1d);
    final Vector result1 =
        edge1.getCheckedStateOnEdge(testVec1, 1e-7d, false);

    assertEquals(0d, result1.getElement(0), 0d);

    final Vector testVec2 =
        VectorFactory.getDenseDefault().createVector2D(
            1e-12d, -1d);
    final Vector result2 =
        edge1.getCheckedStateOnEdge(testVec2, 1e-7d, false);

    assertEquals(1e-12d, result2.getElement(0), 0d);

    final Vector testVec3 =
        VectorFactory.getDenseDefault().createVector2D(
            -1e-6d, -1d);
    final Vector result3 =
        edge1.getCheckedStateOnEdge(testVec3, 1e-7d, false);

    assertEquals(null, result3);

  }

  /**
   * Test around the start of the edge, negative.
   */
  @Test
  public void testPathEdge1_2() {
    final InferredPath path1 =
        TrackingTestUtils.makeTmpPath(graph, true,
            new Coordinate(0, 20d), new Coordinate(
                1d / 10d, 10.11d));

    final PathEdge edge1 =
        Iterables.getFirst(path1.getPathEdges(), null);

    final Vector testVec1 =
        VectorFactory.getDenseDefault().createVector2D(
            1e-12d, -1d);
    final Vector result1 =
        edge1.getCheckedStateOnEdge(testVec1, 1e-7d, false);

    assertEquals(0d, result1.getElement(0), 0d);

    final Vector testVec2 =
        VectorFactory.getDenseDefault().createVector2D(
            -1e-12d, -1d);
    final Vector result2 =
        edge1.getCheckedStateOnEdge(testVec2, 1e-7d, false);

    assertEquals(-1e-12d, result2.getElement(0), 0d);

    final Vector testVec3 =
        VectorFactory.getDenseDefault().createVector2D(
            1e-6d, -1d);
    final Vector result3 =
        edge1.getCheckedStateOnEdge(testVec3, 1e-7d, false);

    assertEquals(null, result3);

  }

  @Test
  public void testPathStateConvert1() {
    final SimpleInferredPath startPath =
        TrackingTestUtils.makeTmpPath(graph, true,
            new Coordinate(0, 0), new Coordinate(10, 0));

    final Matrix covar =
        MatrixFactory.getDefault().copyArray(
            new double[][] { new double[] { 126.56, 8.44 },
                new double[] { 8.44, 0.56 } });
    final MultivariateGaussian startBelief =
        new MultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(-0d, -5d / 30d), covar);
    final SimplePathStateBelief currentBelief =
        SimplePathStateBelief.getPathStateBelief(startPath,
            startBelief);

    final Vector groundLoc =
        AbstractRoadTrackingFilter.getOg().times(
            currentBelief.getGroundState());
    assertEquals("initial state x", 10d,
        groundLoc.getElement(0), 0d);
    assertEquals("initial state y", 0d,
        groundLoc.getElement(1), 0d);

    final InferredPath newPath =
        TrackingTestUtils.makeTmpPath(graph, true,
            new Coordinate(20, -10),
            new Coordinate(10, -10), new Coordinate(10, 0),
            new Coordinate(0, 0));

    final PathStateBelief result =
        newPath.getStateBeliefOnPath(currentBelief);

    assertEquals("distance", -10d, result
        .getGlobalStateBelief().getMean().getElement(0), 0d);
    assertEquals("velocity", 5d / 30d, result
        .getGlobalStateBelief().getMean().getElement(1), 0d);

  }

  @Test
  public void testPathStateConvert2() {
    final SimpleInferredPath startPath =
        TrackingTestUtils.makeTmpPath(graph, false,
            new Coordinate(0, 0), new Coordinate(10, 0));

    final Matrix covar =
        MatrixFactory.getDefault().copyArray(
            new double[][] { new double[] { 126.56, 8.44 },
                new double[] { 8.44, 0.56 } });
    final MultivariateGaussian startBelief =
        new MultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(0d, 1d), covar);
    final SimplePathStateBelief currentBelief =
        SimplePathStateBelief.getPathStateBelief(startPath,
            startBelief);

    final Vector groundLoc =
        AbstractRoadTrackingFilter.getOg().times(
            currentBelief.getGroundState());
    assertEquals("initial state x", 0d,
        groundLoc.getElement(0), 0d);
    assertEquals("initial state y", 0d,
        groundLoc.getElement(1), 0d);

    final InferredPath newPath =
        TrackingTestUtils.makeTmpPath(graph, true,
            new Coordinate(20, -10),
            new Coordinate(10, -10), new Coordinate(10, 0),
            new Coordinate(0, 0));

    final PathStateBelief result =
        newPath.getStateBeliefOnPath(currentBelief);

    assertEquals("distance", -0d, result
        .getGlobalStateBelief().getMean().getElement(0), 0d);
    assertEquals("velocity", -1d, result
        .getGlobalStateBelief().getMean().getElement(1), 0d);

  }

  @Test
  public void testPathStateConvert3() {
    final SimpleInferredPath startPath =
        TrackingTestUtils.makeTmpPath(graph, false,
            new Coordinate(0, 0), new Coordinate(10, 0));

    final Matrix covar =
        MatrixFactory.getDefault().copyArray(
            new double[][] { new double[] { 126.56, 8.44 },
                new double[] { 8.44, 0.56 } });
    final MultivariateGaussian startBelief =
        new MultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(2.5d, 1d), covar);
    final SimplePathStateBelief currentBelief =
        SimplePathStateBelief.getPathStateBelief(startPath,
            startBelief);

    final Vector groundLoc =
        AbstractRoadTrackingFilter.getOg().times(
            currentBelief.getGroundState());
    assertEquals("initial state x", 2.5d,
        groundLoc.getElement(0), 0d);
    assertEquals("initial state y", 0d,
        groundLoc.getElement(1), 0d);

    final InferredPath newPath =
        TrackingTestUtils.makeTmpPath(graph, false,
            new Coordinate(0, 0), new Coordinate(10, 0),
            new Coordinate(10, -10),
            new Coordinate(20, -10));

    final PathStateBelief result =
        newPath.getStateBeliefOnPath(currentBelief);

    assertEquals("distance", 2.5d, result
        .getGlobalStateBelief().getMean().getElement(0), 0d);
    assertEquals("velocity", 1d, result
        .getGlobalStateBelief().getMean().getElement(1), 0d);

  }

  @Test
  public void testPathStateConvert4() {
    final SimpleInferredPath startPath =
        TrackingTestUtils.makeTmpPath(graph, true,
            new Coordinate(10, 0), new Coordinate(0, 0));

    final Matrix covar =
        MatrixFactory.getDefault().copyArray(
            new double[][] { new double[] { 126.56, 8.44 },
                new double[] { 8.44, 0.56 } });
    final MultivariateGaussian startBelief =
        new MultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(-2.5d, 1d), covar);
    final SimplePathStateBelief currentBelief =
        SimplePathStateBelief.getPathStateBelief(startPath,
            startBelief);

    final InferredPath newPath =
        TrackingTestUtils.makeTmpPath(graph, true,
            new Coordinate(20, -10),
            new Coordinate(10, -10), new Coordinate(10, 0),
            new Coordinate(0, 0));

    final PathStateBelief result =
        newPath.getStateBeliefOnPath(currentBelief);

    assertEquals("distance", -2.5d, result
        .getGlobalStateBelief().getMean().getElement(0), 0d);
    assertEquals("velocity", 1d, result
        .getGlobalStateBelief().getMean().getElement(1), 0d);

  }

  /**
   * Path edges are reverse, directions are the same.
   */
  @Test
  public void testPathStateConvert5() {

    final SimpleInferredPath startPath =
        TrackingTestUtils.makeTmpPath(graph, false,
            new Coordinate(0, 0), new Coordinate(10, 0),
            new Coordinate(10, -10),
            new Coordinate(20, -10));
    final Matrix covar =
        MatrixFactory.getDefault().copyArray(
            new double[][] { new double[] { 126.56, 8.44 },
                new double[] { 8.44, 0.56 } });
    final MultivariateGaussian startBelief =
        new MultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(2.5d, 1d), covar);
    final SimplePathStateBelief currentBelief =
        SimplePathStateBelief.getPathStateBelief(startPath,
            startBelief);

    final InferredPath newPath =
        TrackingTestUtils.makeTmpPath(graph, false,
            new Coordinate(10, 0), new Coordinate(0, 0));

    final PathStateBelief result =
        newPath.getStateBeliefOnPath(currentBelief);

    assertEquals("distance", 7.5d, result
        .getGlobalStateBelief().getMean().getElement(0), 0d);
    assertEquals("velocity", -1d, result
        .getGlobalStateBelief().getMean().getElement(1), 0d);

  }

  /**
   * Path & edge are the same, directions are reverse, pos. to neg.
   */
  @Test
  public void testPathStateConvert6() {

    final SimpleInferredPath startPath =
        TrackingTestUtils.makeTmpPath(graph, false,
            new Coordinate(0, 0), new Coordinate(10, 0),
            new Coordinate(10, -10),
            new Coordinate(20, -10));
    final Matrix covar =
        MatrixFactory.getDefault().copyArray(
            new double[][] { new double[] { 126.56, 8.44 },
                new double[] { 8.44, 0.56 } });
    final MultivariateGaussian startBelief =
        new MultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(2.5d, 1d), covar);
    final SimplePathStateBelief currentBelief =
        SimplePathStateBelief.getPathStateBelief(startPath,
            startBelief);

    final InferredPath newPath =
        TrackingTestUtils.makeTmpPath(graph, true,
            new Coordinate(0, 0), new Coordinate(10, 0));

    final PathStateBelief result =
        newPath.getStateBeliefOnPath(currentBelief);

    assertEquals("distance", -7.5d, result
        .getGlobalStateBelief().getMean().getElement(0), 0d);
    assertEquals("velocity", 1d, result
        .getGlobalStateBelief().getMean().getElement(1), 0d);

  }

  /**
   * Path & edge are the same, directions are reverse, neg. to pos.
   */
  @Test
  public void testPathStateConvert7() {

    final SimpleInferredPath startPath =
        TrackingTestUtils.makeTmpPath(graph, true,
            new Coordinate(20, -10),
            new Coordinate(10, -10), new Coordinate(10, 0),
            new Coordinate(0, 0));
    final Matrix covar =
        MatrixFactory.getDefault().copyArray(
            new double[][] { new double[] { 126.56, 8.44 },
                new double[] { 8.44, 0.56 } });
    final MultivariateGaussian startBelief =
        new MultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(-2.5d, 1d), covar);
    final SimplePathStateBelief currentBelief =
        SimplePathStateBelief.getPathStateBelief(startPath,
            startBelief);

    final InferredPath newPath =
        TrackingTestUtils.makeTmpPath(graph, false,
            new Coordinate(10, 0), new Coordinate(0, 0));

    final PathStateBelief result =
        newPath.getStateBeliefOnPath(currentBelief);

    assertEquals("distance", 7.5d, result
        .getGlobalStateBelief().getMean().getElement(0), 0d);
    assertEquals("velocity", 1d, result
        .getGlobalStateBelief().getMean().getElement(1), 0d);

  }

  @Test
  public void testPrediction() throws TimeOrderException {

    final Matrix covar =
        MatrixFactory.getDefault().copyArray(
            new double[][] { new double[] { 126.56, 8.44 },
                new double[] { 8.44, 0.56 } });
    final MultivariateGaussian startBelief =
        new MultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(-10d, -5d / 30d), covar);

    final SimpleInferredPath newPath =
        TrackingTestUtils.makeTmpPath(graph, true,
            new Coordinate(20, -10),
            new Coordinate(10, -10), new Coordinate(10, 0),
            new Coordinate(0, 0));

    Coordinate startCoord = new Coordinate(10, -5);
//    final ProjectedCoordinate obsPoint =
//        GeoUtils.convertToEuclidean(startCoord);
    Vector projPoint = VectorFactory.getDefault().createVector2D(
                startCoord.x, startCoord.y);
    final GpsObservation obs =
        new SimpleObservation("none", new Date(
            System.currentTimeMillis()), startCoord, 
            null, null, null,
            0, null, null);

    final SimplePathStateBelief belief =
        SimplePathStateBelief.getPathStateBelief(newPath,
            startBelief);
    final MultivariateGaussian result =
        Iterables.get(newPath.getPathEdges(), 1).getPriorPredictive(belief, obs);

    assertEquals("distance", -14d, result.getMean()
        .getElement(0), 1d);
    assertTrue("velocity direction", startBelief.getMean()
        .getElement(1) > result.getMean().getElement(1));
  }

}
