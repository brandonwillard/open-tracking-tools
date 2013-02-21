package org.opentrackingtools.statistics.filters.vehicles.road.impl;

import org.testng.annotations.Test;
import org.testng.annotations.BeforeMethod;
import org.testng.AssertJUnit;

import static org.mockito.Mockito.stub;
import static org.mockito.Mockito.mock;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.Random;

import org.geotools.geometry.jts.JTSFactoryFinder;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.edges.impl.SimpleInferredEdge;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.edges.impl.SimplePathEdge;
import org.opentrackingtools.graph.paths.impl.TrackingTestUtils;
import org.opentrackingtools.graph.paths.states.PathStateBelief;
import org.opentrackingtools.graph.paths.states.impl.SimplePathStateBelief;
import org.opentrackingtools.graph.paths.util.PathUtils;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.distributions.impl.AdjMultivariateGaussian;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.impl.VehicleTrackingPLFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.StandardRoadTrackingFilter;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineString;

public class AbstractRoadTrackingFilterTest {

  private VehicleStateInitialParameters vehicleStateInitialParams;
  private AbstractRoadTrackingFilter filter;
  private InferenceGraph graph;

  @BeforeMethod
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
            VehicleTrackingPLFilter.class.getName(), 
            StandardRoadTrackingFilter.class.getName(),
            25, 30, 0l);

    graph = mock(InferenceGraph.class);
    
    /*
     * We're using StandardRoadTrackingFilter, but 
     * only the parts with implementations in 
     * AbstractRoadTrackingFilter
     */
    filter =
        new StandardRoadTrackingFilter(null,
            graph, vehicleStateInitialParams, null);
  }

  /**
   * Check that road-to-ground coordinates is an isomorphism.
   */
  @Test
  public void testGroundProjection1() {

    final Vector mean =
        VectorFactory.getDenseDefault().copyArray(
            new double[] { -86.01050120108039,
                -51.44579396037449 });
    final Vector projMean =
        VectorFactory.getDenseDefault().copyArray(
            new double[] { 324480.0240871321,
                -17.961092165568616, 4306914.231357716,
                -48.208597619441846 });
    final LineString edgeGeom =
        JTSFactoryFinder.getGeometryFactory().createLineString(
            new Coordinate[] { new Coordinate(324470.0865109131,
            4306887.558335339),  new Coordinate(324480.0240871321,
            4306914.231357716), new Coordinate(324487.9070333349,
            4306923.394792204), new Coordinate(324497.3591208192,
            4306927.015912709), new Coordinate(324514.4626894819,
            4306930.933664588)});
    final SimpleInferredEdge infEdge =
        SimpleInferredEdge.getInferredEdge(edgeGeom, null, 680402, graph);
    stub(graph.edgeHasReverse(edgeGeom)).toReturn(false);
    final PathEdge pathEdge =
        SimplePathEdge.getEdge(infEdge, -46d, true);

    AssertJUnit.assertTrue(PathUtils.isIsoMapping(
        mean, projMean, pathEdge));
  }

  /**
   * Check that road prediction from one path to another is consistent.
   */
  @Test
  public void testPrediction1() {

    final InferredPath startPath =
        TrackingTestUtils.makeTmpPath(graph, true,
            new Coordinate(0, 0), new Coordinate(10, 0));

    final Matrix covar =
        MatrixFactory.getDefault().copyArray(
            new double[][] { new double[] { 126.56, 8.44 },
                new double[] { 8.44, 0.56 } });
    final MultivariateGaussian startBelief =
        new AdjMultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(-0d, -5d / 30d), covar);
    final SimplePathStateBelief currentBelief =
        SimplePathStateBelief.getPathStateBelief(startPath,
            startBelief);

    final InferredPath newPath =
        TrackingTestUtils.makeTmpPath(graph, true,
            new Coordinate(10, -10), new Coordinate(10, 0),
            new Coordinate(0, 0));

    final PathStateBelief result =
        filter.predict(currentBelief, newPath);

    AssertJUnit.assertEquals("dist", -5d, result.getGlobalState()
        .getElement(0), 0d);
    AssertJUnit.assertEquals("new path", newPath, result.getPath());

    final MultivariateGaussian startBelief2 =
        new AdjMultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(-0d, 5d / 30d), covar);
    final SimplePathStateBelief currentBelief2 =
        SimplePathStateBelief.getPathStateBelief(startPath,
            startBelief2);

    final PathStateBelief result2 =
        filter.predict(currentBelief2, newPath);

    AssertJUnit.assertEquals("dist 2", -15d, result2.getGlobalState()
        .getElement(0), 0d);
    AssertJUnit.assertEquals("new path 2", newPath, result2.getPath());

  }

  @Test
  public void testGroundStateTransSampling() {
    final Random rng = new Random(123456789);
    final MultivariateGaussian.SufficientStatistic ss =
        new MultivariateGaussian.SufficientStatistic();
    final Vector zeros =
        VectorFactory.getDenseDefault().createVector(4);
    for (int i = 0; i < 100000; i++) {
      final Vector smpl2 =
          filter.sampleStateTransDist(zeros, rng);
      ss.update(smpl2);
    }

    final Matrix varErr =
        filter.getOffRoadStateTransCovar().minus(
            ss.getCovariance());

    final Matrix leftInv =
        filter.getCovarianceFactorLeftInv(false);
    final Matrix rfact =
        leftInv.times(ss.getCovariance()).times(
            leftInv.transpose());

    final Matrix factErr = filter.getQg().minus(rfact);

    AssertJUnit.assertEquals(0, varErr.normFrobenius(), 5e-1d);
    AssertJUnit.assertEquals(0, factErr.normFrobenius(), 1e-5d);

  }

  @Test
  public void testRoadStateTransSampling() {
    final Random rng = new Random(123456789);
    final MultivariateGaussian.SufficientStatistic ss =
        new MultivariateGaussian.SufficientStatistic();
    final Vector zeros =
        VectorFactory.getDenseDefault().createVector(2);
    for (int i = 0; i < 100000; i++) {
      final Vector smpl2 =
          filter.sampleStateTransDist(zeros, rng);
      ss.update(smpl2);
    }

    final Matrix varErr =
        filter.getOnRoadStateTransCovar().minus(
            ss.getCovariance());

    final Matrix leftInv =
        filter.getCovarianceFactorLeftInv(true);
    final Matrix rfact =
        leftInv.times(ss.getCovariance()).times(
            leftInv.transpose());

    final Matrix factErr = filter.getQr().minus(rfact);

    AssertJUnit.assertTrue(varErr.normFrobenius() < 1d);
    AssertJUnit.assertTrue(factErr.normFrobenius() < 1e-5d);

  }

}
