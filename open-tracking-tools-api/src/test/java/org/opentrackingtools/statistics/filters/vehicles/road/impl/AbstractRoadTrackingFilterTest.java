package org.opentrackingtools.statistics.filters.vehicles.road.impl;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.Random;

import org.junit.Before;
import org.junit.Test;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.edges.impl.SimpleInferredEdge;
import org.opentrackingtools.graph.otp.impl.OtpGraph;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.edges.impl.SimplePathEdge;
import org.opentrackingtools.graph.paths.impl.SimpleInferredPath;
import org.opentrackingtools.graph.paths.impl.TrackingTestUtils;
import org.opentrackingtools.graph.paths.states.PathStateBelief;
import org.opentrackingtools.graph.paths.states.impl.SimplePathStateBelief;
import org.opentrackingtools.graph.paths.util.PathUtils;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.impl.VehicleTrackingPLFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.StandardRoadTrackingFilter;
import org.opentripplanner.common.geometry.GeometryUtils;
import org.opentripplanner.routing.edgetype.PlainStreetEdge;
import org.opentripplanner.routing.vertextype.StreetVertex;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineString;

public class AbstractRoadTrackingFilterTest {

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

    /*
     * We're using StandardRoadTrackingFilter, but 
     * only the parts with implementations in 
     * AbstractRoadTrackingFilter
     */
    filter =
        new StandardRoadTrackingFilter(
            vehicleStateInitialParams.getObsCov(),
            vehicleStateInitialParams.getOffRoadStateCov(),
            vehicleStateInitialParams.getOnRoadStateCov(),
            vehicleStateInitialParams.getInitialObsFreq());
    //        mock(AbstractRoadTrackingFilter.class);
    //    filter.setObsCovar(
    //        MatrixFactory.getDenseDefault().createDiagonal(
    //        vehicleStateInitialParams.getObsCov()));
    //    filter.setOnRoadStateTransCovar(
    //        MatrixFactory.getDenseDefault().createDiagonal(
    //        vehicleStateInitialParams.getOnRoadStateCov()));
    //    filter.setOffRoadStateTransCovar(
    //        MatrixFactory.getDenseDefault().createDiagonal(
    //        vehicleStateInitialParams.getOffRoadStateCov()));
    //    filter.setCurrentTimeDiff(
    //        vehicleStateInitialParams.getInitialObsFreq());

    graph = mock(OtpGraph.class);
  }

  /**
   * Check that road-to-ground coordinates is an isomorphism.
   */
  @Test
  public void test1() {

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
        GeometryUtils.makeLineString(324470.0865109131,
            4306887.558335339, 324480.0240871321,
            4306914.231357716, 324487.9070333349,
            4306923.394792204, 324497.3591208192,
            4306927.015912709, 324514.4626894819,
            4306930.933664588);
    final StreetVertex v1 = mock(StreetVertex.class);
    final PlainStreetEdge edge =
        new PlainStreetEdge(v1, v1, edgeGeom, "test", 0d,
            null, false);
    final SimpleInferredEdge infEdge =
        SimpleInferredEdge.getInferredEdge(edge.getGeometry(), edge, 680402, graph);
    final PathEdge pathEdge =
        SimplePathEdge.getEdge(infEdge, -46d, true);

    assertTrue(PathUtils.isIsoMapping(
        mean, projMean, pathEdge));
  }

  /**
   * Check that road prediction from one path to another is consistent.
   */
  @Test
  public void test2() {

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

    final SimpleInferredPath newPath =
        TrackingTestUtils.makeTmpPath(graph, true,
            new Coordinate(10, -10), new Coordinate(10, 0),
            new Coordinate(0, 0));

    final PathStateBelief result =
        filter.predict(currentBelief, newPath);

    assertEquals("dist", -5d, result.getGlobalState()
        .getElement(0), 0d);
    assertEquals("new path", newPath, result.getPath());

    final MultivariateGaussian startBelief2 =
        new MultivariateGaussian(VectorFactory.getDefault()
            .createVector2D(-0d, 5d / 30d), covar);
    final SimplePathStateBelief currentBelief2 =
        SimplePathStateBelief.getPathStateBelief(startPath,
            startBelief2);

    final PathStateBelief result2 =
        filter.predict(currentBelief2, newPath);

    assertEquals("dist 2", -15d, result2.getGlobalState()
        .getElement(0), 0d);
    assertEquals("new path 2", newPath, result2.getPath());

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

    assertTrue(varErr.normFrobenius() < 1d);
    assertTrue(factErr.normFrobenius() < 1e-5d);

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

    assertTrue(varErr.normFrobenius() < 1d);
    assertTrue(factErr.normFrobenius() < 1e-5d);

  }

}
