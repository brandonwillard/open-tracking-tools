package org.openplans.tools.tracking.impl;

import static org.junit.Assert.*;
import static org.mockito.Mockito.mock;

import java.util.Date;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import org.junit.Before;
import org.junit.Test;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.graph.paths.PathState;
import org.openplans.tools.tracking.impl.graph.paths.PathStateBelief;
import org.openplans.tools.tracking.impl.statistics.filters.particle_learning.VehicleTrackingPLFilter;
import org.openplans.tools.tracking.impl.statistics.filters.road_tracking.AbstractRoadTrackingFilter;
import org.openplans.tools.tracking.impl.statistics.filters.road_tracking.StandardRoadTrackingFilter;
import org.openplans.tools.tracking.impl.util.OtpGraph;

import com.google.common.collect.Iterables;
import com.vividsolutions.jts.geom.Coordinate;

public class InferredPathTest {

  private VehicleStateInitialParameters vehicleStateInitialParams;
  private AbstractRoadTrackingFilter<?> filter;
  private OtpGraph graph;

  @Before
  public void setUp() throws Exception {
    
    vehicleStateInitialParams = new VehicleStateInitialParameters(
        VectorFactory.getDefault().createVector2D(100d, 100d), 20,
        VectorFactory.getDefault().createVector1D(0.000625), 20,
        VectorFactory.getDefault().createVector2D(0.000625, 0.000625), 20,
        VectorFactory.getDefault().createVector2D(5d, 95d),
        VectorFactory.getDefault().createVector2D(95d, 5d), 
        VehicleTrackingPLFilter.class.getName(),
        25, 30, 0l);
    
    filter = new StandardRoadTrackingFilter(
        vehicleStateInitialParams.getObsCov(),
        vehicleStateInitialParams.getOffRoadStateCov(),
        vehicleStateInitialParams.getOnRoadStateCov(),
        vehicleStateInitialParams.getInitialObsFreq());

    graph = mock(OtpGraph.class);
  }

  /**
   * Test conversion from a positive state
   * to a negative one with an opposite geom.
   */
  @Test
  public void testGetStateOnPath() {
    
    InferredPath startPath = TrackingTestUtils.makeTmpPath(graph, 
        false, 
        new Coordinate(0, 0),
        new Coordinate(0, 60)
      );
    
    PathState startState = PathState.getPathState(startPath,
        VectorFactory.getDefault().createVector2D(12d, -1));
    
    InferredPath newPath = TrackingTestUtils.makeTmpPath(graph, 
        true, 
        new Coordinate(0, 60),
        new Coordinate(0, 0)
      );
    
    final Vector result = newPath.getStateOnPath(startState);
    
    assertEquals("dist", -12d, result.getElement(0), 0d);
    assertEquals("dist", 1d, result.getElement(1), 0d);
    
  }
  
  @Test
  public void testPrediction() throws TimeOrderException {
    
    Matrix covar = MatrixFactory.getDefault().copyArray(
        new double[][] {
            new double[] {126.56,8.44}, 
            new double[] {8.44, 0.56}
            }); 
    MultivariateGaussian startBelief = new MultivariateGaussian(
        VectorFactory.getDefault().createVector2D(-10d, -5d/30d), covar);
    
    InferredPath newPath = TrackingTestUtils.makeTmpPath(graph, 
        true, 
        new Coordinate(20, -10),
        new Coordinate(10, -10),
        new Coordinate(10, 0), 
        new Coordinate(0, 0)
      );
    
    Observation obs = Observation.createObservation("none", 
        new Date(System.currentTimeMillis()), 
        new Coordinate(10, -5), 0d, 0d, 0d);
    
    PathStateBelief belief = PathStateBelief.getPathStateBelief(newPath,
        startBelief);
    MultivariateGaussian result = newPath.predict(belief, 
        obs, Iterables.get(newPath.getEdges(), 1));
    
    assertEquals("distance", -14d, result.getMean().getElement(0), 1d);
    assertTrue("velocity direction", 
        startBelief.getMean().getElement(1) 
          > result.getMean().getElement(1));
  }
  
  /**
   * Test around the start of the edge, positive.
   */
  @Test
  public void testPathEdge1() {
    InferredPath path1 = TrackingTestUtils.makeTmpPath(graph, 
        false, 
        new Coordinate(0, 10.11d),
        new Coordinate(10.11d, 20d));
    
    final PathEdge edge1 = Iterables.getFirst(path1.getEdges(), null);
    
    final Vector testVec1 = VectorFactory.getDenseDefault().
        createVector2D(-1e-12d, -1d);
    final Vector result1 = edge1.getCheckedStateOnEdge(
        testVec1, 1e-7d, false);
    
    assertEquals(0d, result1.getElement(0), 0d);
    
    final Vector testVec2 = VectorFactory.getDenseDefault().
        createVector2D(1e-12d, -1d);
    final Vector result2 = edge1.getCheckedStateOnEdge(
        testVec2, 1e-7d, false);
    
    assertEquals(1e-12d, result2.getElement(0), 0d);
    
    final Vector testVec3 = VectorFactory.getDenseDefault().
        createVector2D(-1e-6d, -1d);
    final Vector result3 = edge1.getCheckedStateOnEdge(
        testVec3, 1e-7d, false);
    
    assertEquals(null, result3);
    
  }
  
  /**
   * Test around the start of the edge, negative.
   */
  @Test
  public void testPathEdge1_2() {
    InferredPath path1 = TrackingTestUtils.makeTmpPath(graph, 
        true, 
        new Coordinate(0, 20d),
        new Coordinate(1d/10d, 10.11d)
        );
    
    final PathEdge edge1 = Iterables.getFirst(path1.getEdges(), null);
    
    final Vector testVec1 = VectorFactory.getDenseDefault().
        createVector2D(1e-12d, -1d);
    final Vector result1 = edge1.getCheckedStateOnEdge(
        testVec1, 1e-7d, false);
    
    assertEquals(0d, result1.getElement(0), 0d);
    
    final Vector testVec2 = VectorFactory.getDenseDefault().
        createVector2D(-1e-12d, -1d);
    final Vector result2 = edge1.getCheckedStateOnEdge(
        testVec2, 1e-7d, false);
    
    assertEquals(-1e-12d, result2.getElement(0), 0d);
    
    final Vector testVec3 = VectorFactory.getDenseDefault().
        createVector2D(1e-6d, -1d);
    final Vector result3 = edge1.getCheckedStateOnEdge(
        testVec3, 1e-7d, false);
    
    assertEquals(null, result3);
    
  }
  
  
  @Test(expected=IllegalStateException.class)
  public void testBadConversion() {
    InferredPath startPath = TrackingTestUtils.makeTmpPath(graph, 
        true, 
        new Coordinate(20, -10),
        new Coordinate(10, -10));
    
    Matrix covar = MatrixFactory.getDefault().copyArray(
        new double[][] {
            new double[] {126.56,8.44}, 
            new double[] {8.44, 0.56}
            }); 
    MultivariateGaussian startBelief = new MultivariateGaussian(
        VectorFactory.getDefault().createVector2D(-0d, -5d/30d), covar);
    PathStateBelief currentBelief = PathStateBelief.
        getPathStateBelief(startPath, startBelief);
    
    InferredPath newPath = TrackingTestUtils.makeTmpPath(graph, 
        true, 
        new Coordinate(20, -10),
        new Coordinate(10, -10),
        new Coordinate(10, 0), 
        new Coordinate(0, 0) 
        );
    
    PathStateBelief result = newPath.getStateBeliefOnPath(currentBelief);
    
    
  }
  
  @Test
  public void testPathStateConvert1() {
    InferredPath startPath = TrackingTestUtils.makeTmpPath(graph, 
        true, 
        new Coordinate(0, 0),
        new Coordinate(10, 0));
    
    Matrix covar = MatrixFactory.getDefault().copyArray(
        new double[][] {
            new double[] {126.56,8.44}, 
            new double[] {8.44, 0.56}
            }); 
    MultivariateGaussian startBelief = new MultivariateGaussian(
        VectorFactory.getDefault().createVector2D(-0d, -5d/30d), covar);
    PathStateBelief currentBelief = PathStateBelief.
        getPathStateBelief(startPath, startBelief);
    
    final Vector groundLoc = AbstractRoadTrackingFilter.getOg().times(
        currentBelief.getGroundState());
    assertEquals("initial state x", 10d, groundLoc.getElement(0), 0d);
    assertEquals("initial state y", 0d, groundLoc.getElement(1), 0d);
    
    InferredPath newPath = TrackingTestUtils.makeTmpPath(graph, 
        true, 
        new Coordinate(20, -10),
        new Coordinate(10, -10),
        new Coordinate(10, 0), 
        new Coordinate(0, 0) 
        );
    
    PathStateBelief result = newPath.getStateBeliefOnPath(currentBelief);
    
    assertEquals("distance", -10d, 
        result.getGlobalStateBelief().getMean().getElement(0), 0d);
    assertEquals("velocity", 5d/30d, 
        result.getGlobalStateBelief().getMean().getElement(1), 0d);
    
  }
  
  @Test
  public void testPathStateConvert2() {
    InferredPath startPath = TrackingTestUtils.makeTmpPath(graph, 
        false, 
        new Coordinate(0, 0),
        new Coordinate(10, 0));
    
    Matrix covar = MatrixFactory.getDefault().copyArray(
        new double[][] {
            new double[] {126.56,8.44}, 
            new double[] {8.44, 0.56}
            }); 
    MultivariateGaussian startBelief = new MultivariateGaussian(
        VectorFactory.getDefault().createVector2D(0d, 1d), covar);
    PathStateBelief currentBelief = PathStateBelief.
        getPathStateBelief(startPath, startBelief);
    
    final Vector groundLoc = AbstractRoadTrackingFilter.getOg().times(
        currentBelief.getGroundState());
    assertEquals("initial state x", 0d, groundLoc.getElement(0), 0d);
    assertEquals("initial state y", 0d, groundLoc.getElement(1), 0d);
    
    InferredPath newPath = TrackingTestUtils.makeTmpPath(graph, 
        true, 
        new Coordinate(20, -10),
        new Coordinate(10, -10),
        new Coordinate(10, 0), 
        new Coordinate(0, 0) 
        );
    
    PathStateBelief result = newPath.getStateBeliefOnPath(currentBelief);
    
    assertEquals("distance", -0d, 
        result.getGlobalStateBelief().getMean().getElement(0), 0d);
    assertEquals("velocity", -1d, 
        result.getGlobalStateBelief().getMean().getElement(1), 0d);
    
  }
  
  @Test
  public void testPathStateConvert3() {
    InferredPath startPath = TrackingTestUtils.makeTmpPath(graph, 
        false, 
        new Coordinate(0, 0),
        new Coordinate(10, 0));
    
    Matrix covar = MatrixFactory.getDefault().copyArray(
        new double[][] {
            new double[] {126.56,8.44}, 
            new double[] {8.44, 0.56}
            }); 
    MultivariateGaussian startBelief = new MultivariateGaussian(
        VectorFactory.getDefault().createVector2D(2.5d, 1d), covar);
    PathStateBelief currentBelief = PathStateBelief.
        getPathStateBelief(startPath, startBelief);
    
    final Vector groundLoc = AbstractRoadTrackingFilter.getOg().times(
        currentBelief.getGroundState());
    assertEquals("initial state x", 2.5d, groundLoc.getElement(0), 0d);
    assertEquals("initial state y", 0d, groundLoc.getElement(1), 0d);
    
    InferredPath newPath = TrackingTestUtils.makeTmpPath(graph, 
        false, 
        new Coordinate(0, 0),
        new Coordinate(10, 0), 
        new Coordinate(10, -10),
        new Coordinate(20, -10)
        );
    
    PathStateBelief result = newPath.getStateBeliefOnPath(currentBelief);
    
    assertEquals("distance", 2.5d, 
        result.getGlobalStateBelief().getMean().getElement(0), 0d);
    assertEquals("velocity", 1d, 
        result.getGlobalStateBelief().getMean().getElement(1), 0d);
    
  }
  
  @Test
  public void testPathStateConvert4() {
    InferredPath startPath = TrackingTestUtils.makeTmpPath(graph, 
        true, 
        new Coordinate(10, 0),
        new Coordinate(0, 0)
    );
    
    Matrix covar = MatrixFactory.getDefault().copyArray(
        new double[][] {
            new double[] {126.56,8.44}, 
            new double[] {8.44, 0.56}
            }); 
    MultivariateGaussian startBelief = new MultivariateGaussian(
        VectorFactory.getDefault().createVector2D(-2.5d, 1d), covar);
    PathStateBelief currentBelief = PathStateBelief.
        getPathStateBelief(startPath, startBelief);
    
    InferredPath newPath = TrackingTestUtils.makeTmpPath(graph, 
        true, 
        new Coordinate(20, -10),
        new Coordinate(10, -10),
        new Coordinate(10, 0), 
        new Coordinate(0, 0)
        );
    
    PathStateBelief result = newPath.getStateBeliefOnPath(currentBelief);
    
    assertEquals("distance", -2.5d, 
        result.getGlobalStateBelief().getMean().getElement(0), 0d);
    assertEquals("velocity", 1d, 
        result.getGlobalStateBelief().getMean().getElement(1), 0d);
    
  }
  
  /**
   * Path edges are reverse, directions are the same.
   */
  @Test
  public void testPathStateConvert5() {
    
    InferredPath startPath = TrackingTestUtils.makeTmpPath(graph, 
        false, 
        new Coordinate(0, 0),
        new Coordinate(10, 0), 
        new Coordinate(10, -10),
        new Coordinate(20, -10)
        );
    Matrix covar = MatrixFactory.getDefault().copyArray(
        new double[][] {
            new double[] {126.56,8.44}, 
            new double[] {8.44, 0.56}
            }); 
    MultivariateGaussian startBelief = new MultivariateGaussian(
        VectorFactory.getDefault().createVector2D(2.5d, 1d), covar);
    PathStateBelief currentBelief = PathStateBelief.
        getPathStateBelief(startPath, startBelief);
    
    InferredPath newPath = TrackingTestUtils.makeTmpPath(graph, 
        false, 
        new Coordinate(10, 0),
        new Coordinate(0, 0)
    );
    
    PathStateBelief result = newPath.getStateBeliefOnPath(currentBelief);
    
    assertEquals("distance", 7.5d, 
        result.getGlobalStateBelief().getMean().getElement(0), 0d);
    assertEquals("velocity", -1d, 
        result.getGlobalStateBelief().getMean().getElement(1), 0d);
    
  }
  
  /**
   * Path & edge are the same, directions are reverse, pos. to neg.
   */
  @Test
  public void testPathStateConvert6() {
    
    InferredPath startPath = TrackingTestUtils.makeTmpPath(graph, 
        false, 
        new Coordinate(0, 0),
        new Coordinate(10, 0), 
        new Coordinate(10, -10),
        new Coordinate(20, -10)
        );
    Matrix covar = MatrixFactory.getDefault().copyArray(
        new double[][] {
            new double[] {126.56,8.44}, 
            new double[] {8.44, 0.56}
            }); 
    MultivariateGaussian startBelief = new MultivariateGaussian(
        VectorFactory.getDefault().createVector2D(2.5d, 1d), covar);
    PathStateBelief currentBelief = PathStateBelief.
        getPathStateBelief(startPath, startBelief);
    
    InferredPath newPath = TrackingTestUtils.makeTmpPath(graph, 
        true, 
        new Coordinate(0, 0),
        new Coordinate(10, 0)
    );
    
    PathStateBelief result = newPath.getStateBeliefOnPath(currentBelief);
    
    assertEquals("distance", -7.5d, 
        result.getGlobalStateBelief().getMean().getElement(0), 0d);
    assertEquals("velocity", 1d, 
        result.getGlobalStateBelief().getMean().getElement(1), 0d);
    
  }
  
  /**
   * Path & edge are the same, directions are reverse, neg. to pos.
   */
  @Test
  public void testPathStateConvert7() {
    
    InferredPath startPath = TrackingTestUtils.makeTmpPath(graph, 
        true, 
        new Coordinate(20, -10),
        new Coordinate(10, -10),
        new Coordinate(10, 0), 
        new Coordinate(0, 0)
        );
    Matrix covar = MatrixFactory.getDefault().copyArray(
        new double[][] {
            new double[] {126.56,8.44}, 
            new double[] {8.44, 0.56}
            }); 
    MultivariateGaussian startBelief = new MultivariateGaussian(
        VectorFactory.getDefault().createVector2D(-2.5d, 1d), covar);
    PathStateBelief currentBelief = PathStateBelief.
        getPathStateBelief(startPath, startBelief);
    
    InferredPath newPath = TrackingTestUtils.makeTmpPath(graph, 
        false, 
        new Coordinate(10, 0),
        new Coordinate(0, 0)
    );
    
    PathStateBelief result = newPath.getStateBeliefOnPath(currentBelief);
    
    assertEquals("distance", 7.5d, 
        result.getGlobalStateBelief().getMean().getElement(0), 0d);
    assertEquals("velocity", 1d, 
        result.getGlobalStateBelief().getMean().getElement(1), 0d);
    
  }

}
