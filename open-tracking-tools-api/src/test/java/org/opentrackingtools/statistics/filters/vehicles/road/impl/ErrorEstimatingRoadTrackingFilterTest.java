package org.opentrackingtools.statistics.filters.vehicles.road.impl;

import org.testng.annotations.Test;
import org.testng.annotations.BeforeMethod;
import org.testng.AssertJUnit;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.stub;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.distribution.InverseWishartDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.edges.impl.SimplePathEdge;
import org.opentrackingtools.graph.paths.impl.SimpleInferredPath;
import org.opentrackingtools.graph.paths.impl.TrackingTestUtils;
import org.opentrackingtools.graph.paths.states.PathStateBelief;
import org.opentrackingtools.graph.paths.states.impl.SimplePathStateBelief;
import org.opentrackingtools.impl.TimeOrderException;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.distributions.impl.AdjMultivariateGaussian;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.impl.VehicleTrackingPLFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.ErrorEstimatingRoadTrackingFilter;
import org.opentrackingtools.statistics.impl.StatisticsUtil;

import com.beust.jcommander.internal.Lists;
import com.vividsolutions.jts.geom.Coordinate;

@Test(threadPoolSize=3, invocationCount=1)
public class ErrorEstimatingRoadTrackingFilterTest {

  private VehicleStateInitialParameters vehicleStateInitialParams;
  private ErrorEstimatingRoadTrackingFilter filter;
  private InferenceGraph graph;

  private Vector
      sampleTransition(Vector state,
        AbstractRoadTrackingFilter filter, Matrix Qr,
        Random rng) {
    final int dim = state.getDimensionality();
    final Matrix sampleCovChol =
        StatisticsUtil.rootOfSemiDefinite(Qr);
    final Vector qSmpl =
        MultivariateGaussian.sample(VectorFactory
            .getDenseDefault().createVector(dim / 2),
            sampleCovChol, rng);
    final Vector stateSmpl =
        state.plus(filter.getCovarianceFactor(dim == 2)
            .times(qSmpl));
    
    return stateSmpl;
  }

  @BeforeMethod
  public void setUp() throws Exception {

    vehicleStateInitialParams =
        new VehicleStateInitialParameters(VectorFactory
            .getDefault().createVector2D(100d, 100d), 20,
            VectorFactory.getDefault().createVector1D(
                6.25e-4), 30, VectorFactory.getDefault()
                .createVector2D(6.25e-4, 6.25e-4), 30,
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
        new ErrorEstimatingRoadTrackingFilter(
            null, graph, vehicleStateInitialParams,
            new Random(1234567890));

  }

  private void simplePathTest(InferredPath startPath,
    MultivariateGaussian startState, int iterations) {

    final boolean isOnRoad = !startPath.isNullPath();
    filter.getCovarianceFactor(isOnRoad);

    final Matrix trueCovar =
        startState.getCovariance().clone();

    PathStateBelief currentState = 
        startPath.getStateBeliefOnPath(startState);
    PathStateBelief trueState = currentState.clone();
    /*
     * Let's start this from the edge we're on.
     */
    currentState = currentState.getTruncatedPathStateBelief();
    currentState = SimplePathStateBelief.getPathStateBelief(
        SimpleInferredPath.getInferredPath(currentState.getEdge()),
            currentState.getLocalStateBelief());

    final Random rng = new Random(987654321);

    /*
     * Produce a true state and observation.
     */

    final Matrix trueStateCov;
    if (isOnRoad) {
      trueStateCov =
          filter.getOnRoadStateVariancePrior().getMean()
              .clone();
    } else {
      trueStateCov =
          filter.getOffRoadStateVariancePrior().getMean()
              .clone();
    }

    final Matrix trueObsCov =
        MatrixFactory.getDiagonalDefault().createDiagonal(
            this.vehicleStateInitialParams.getObsCov());

    final GpsObservation obs = mock(GpsObservation.class);
    stub(obs.getProjectedPoint()).toReturn(
        AbstractRoadTrackingFilter.getOg().times(
            currentState.getGroundState()));
    final Matrix obsCovarChol =
        StatisticsUtil.rootOfSemiDefinite(trueObsCov);

    final VehicleState state = mock(VehicleState.class);
    stub(state.getBelief()).toReturn(currentState);


    final NumberFormat formatter =
        new DecimalFormat("##.#######");

    final MultivariateGaussian.SufficientStatistic samplesSS =
        new MultivariateGaussian.SufficientStatistic();
    final MultivariateGaussian.SufficientStatistic residualsSS =
        new MultivariateGaussian.SufficientStatistic();

    InferredPath currentEstPath = startPath;

    for (int i = 0; i < iterations; i++) {
      System.out.println("i=" + i);
      System.out.println("\tobs="
          + obs.getProjectedPoint().toString(formatter));
      System.out.println("\ttrueState="
          + trueState.getRawState().toString(formatter));
      //      System.out.println("\ttrueStateCov=" 
      //        + trueState.getCovariance().convertToVector().toString(formatter));

      /*
       * Perform Kalman steps
       */
      final PathEdge presentEdge = currentState.getEdge();
      final PathEdge trueEdge = trueState.getEdge();
      if (!presentEdge.isNullEdge()) {
        final List<PathEdge> newEdges =
            Lists.newArrayList();
        
        final int trueIdx = ((SimpleInferredPath) startPath).getInferredEdges().indexOf(
            trueEdge.getInferredEdge());
        final int presentIdx = ((SimpleInferredPath) startPath).getInferredEdges().indexOf(
            presentEdge.getInferredEdge());
        List<PathEdge> subList;
        final double direction;
        if (presentIdx < trueIdx) {
          subList = Lists.newArrayList(startPath.getPathEdges().subList(presentIdx, trueIdx));
          subList.add(trueEdge);
          direction = 1d;
        } else if (presentIdx > trueIdx) {
          subList = Lists.newArrayList(startPath.getPathEdges().subList(trueIdx, presentIdx));
          subList.add(presentEdge);
          Collections.reverse(subList); 
          direction = -1d;
        } else {
          subList = Collections.singletonList(presentEdge);
          direction = presentEdge.isBackward() ? -1d : 1d;
        }
        
        if (subList.isEmpty())
          subList.add(presentEdge);
        
        PathEdge lastEdge = null;
        for (final PathEdge edge : subList) {
          PathEdge newEdge = SimplePathEdge.getEdge(
              edge.getInferredEdge(),
              direction * (
                  lastEdge == null ? 0d : lastEdge.getLength() + lastEdge.getDistToStartOfEdge()),
              direction < 0d);
          newEdges.add(newEdge);
          lastEdge = newEdge; 
        }
        currentEstPath =
            SimpleInferredPath.getInferredPath(newEdges,
                direction < 0d);
      } else {
        currentEstPath = startPath;
      }

      final PathStateBelief predictedState =
          filter.predict(currentState, currentEstPath);
      final PathStateBelief updatedState =
          filter.measure(predictedState,
              obs.getProjectedPoint(),
              predictedState.getEdge());

      System.out.println("\tcurrentState="
          + currentState.getRawState().toString(formatter));
      System.out.println("\tcurrentStateCov="
          + currentState.getCovariance().convertToVector()
              .toString(formatter));

      final InverseWishartDistribution priorQ =
          isOnRoad ? filter.getOnRoadStateVariancePrior()
              .clone() : filter
              .getOffRoadStateVariancePrior().clone();

      final Matrix A =
          isOnRoad ? filter.getRoadModel().getA() : filter
              .getGroundModel().getA();

      System.out.println("\tQMean="
          + priorQ.getMean().convertToVector()
              .toString(formatter));

      System.out.println("\tQ="
          + (isOnRoad ? filter.getQr().convertToVector()
              .toString(formatter) : filter.getQg()
              .convertToVector().toString(formatter)));

      final InverseWishartDistribution priorObs =
          filter.getObsVariancePrior().clone();

      System.out.println("\tobsMean="
          + priorObs.getMean().convertToVector()
              .toString(formatter));

      /*
       * Update parameters
       */
      filter.update(state, obs, updatedState,
          predictedState, rng);
      currentState = updatedState;

      stub(state.getBelief()).toReturn(updatedState);

      final InverseWishartDistribution posteriorObs =
          filter.getObsVariancePrior();

      System.out.println("\tobsMean="
          + filter.getObsVariancePrior().getMean()
              .convertToVector().toString(formatter));
      System.out.println("\t\tobsDiff="
          + posteriorObs.getMean()
              .minus(priorObs.getMean()).convertToVector()
              .toString(formatter));

      final Vector statesDiff = trueState.minus(currentState);
      residualsSS.update(statesDiff);
      System.out.println("\t\ttrueStateDiff="
          + statesDiff.convertToVector()
              .toString(formatter));

      final InverseWishartDistribution posteriorQ =
          isOnRoad ? filter.getOnRoadStateVariancePrior()
              : filter.getOffRoadStateVariancePrior();

      System.out.println("\tQMean="
          + posteriorQ.getMean().convertToVector()
              .toString(formatter));

      System.out.println("\t\tQMeanDiff="
          + posteriorQ.getMean().minus(priorQ.getMean())
              .convertToVector().toString(formatter));

      System.out.println("\tQ="
          + (isOnRoad ? filter.getQr().convertToVector()
              .toString(formatter) : filter.getQg()
              .convertToVector().toString(formatter)));
      if (filter.getCurrentStateSample() != null) {
        System.out.println("\tprevSampleProj="
            + A.times(
                filter.getPrevStateSample().getRawState())
                .toString(formatter));
        System.out.println("\tcurrSample    ="
            + filter.getCurrentStateSample().getRawState()
                .toString(formatter));
        final Vector sampleDiff =
            filter
                .getCurrentStateSample()
                .getRawState()
                .minus(
                    A.times(filter.getPrevStateSample()
                        .getRawState()));
        samplesSS.update(sampleDiff);
        System.out.println("\t\tcurrSampleDiff ="
            + sampleDiff.toString(formatter));
      }

      /*
       * Update true state
       */
      Vector newStateMean =
          A.times(trueState.getRawStateBelief().getMean());
      newStateMean =
          this.sampleTransition(newStateMean, filter,
              trueStateCov, rng);
      trueState = 
          startPath.getStateBeliefOnPath(
              trueState.getPath().getStateBeliefOnPath(
                new AdjMultivariateGaussian(newStateMean,
                    trueCovar)));
      stub(obs.getProjectedPoint()).toReturn(
          MultivariateGaussian.sample(
              AbstractRoadTrackingFilter.getOg().times(
                  trueState.getGroundState()),
              obsCovarChol, rng));
      
      final Matrix obsMean =
          filter.getObsVariancePrior().getMean();
      AssertJUnit.assertTrue(obsMean.minus(trueObsCov).normFrobenius()
          / obsMean.normFrobenius() <= 0.4d);
  
      final Matrix stateTransMean =
          isOnRoad ? filter.getOnRoadStateVariancePrior()
              .getMean() : filter
              .getOffRoadStateVariancePrior().getMean();
      final Matrix stateTransDiff =
          stateTransMean.minus(trueStateCov);
  
      AssertJUnit.assertTrue(stateTransDiff.normFrobenius()
          / stateTransMean.normFrobenius() <= 0.5d);

    }

    System.out.println("samplesMean="
        + samplesSS.getMean().convertToVector()
            .toString(formatter));
    System.out.println("samplesCov="
        + samplesSS.getCovariance().toString(formatter));

    System.out.println("residualMean="
        + residualsSS.getMean().convertToVector()
            .toString(formatter));
    System.out.println("residualCov="
        + residualsSS.getCovariance().toString(formatter));
    
    final Matrix obsMean =
        filter.getObsVariancePrior().getMean();
    AssertJUnit.assertTrue(obsMean.minus(trueObsCov).normFrobenius()
        / obsMean.normFrobenius() <= 0.4d);

    final Matrix stateTransMean =
        isOnRoad ? filter.getOnRoadStateVariancePrior()
            .getMean() : filter
            .getOffRoadStateVariancePrior().getMean();
    final Matrix stateTransDiff =
        stateTransMean.minus(trueStateCov);

    AssertJUnit.assertTrue(stateTransDiff.normFrobenius()
        / stateTransMean.normFrobenius() <= 0.4d);

  }

  /**
   * @throws TimeOrderException
   */
  @Test
  public void testGroundStateTransCovLearning()
      throws TimeOrderException {
    final InferredPath startPath =
        SimpleInferredPath.getNullPath();
    final Matrix covFactor =
        filter.getCovarianceFactor(false);
    final Matrix covar =
        covFactor.times(
            MatrixFactory.getDefault().createDiagonal(
                this.vehicleStateInitialParams
                    .getOffRoadStateCov())).times(
            covFactor.transpose());

    final MultivariateGaussian startState =
        new AdjMultivariateGaussian(VectorFactory.getDefault()
            .copyArray(new double[] { 0d, 1d, 0d, 1d }),
            covar);

    simplePathTest(startPath, startState, 10000);
  }

  /**
   * Simple, long straight-line path from negative to positive.
   * 
   * @throws TimeOrderException
   */
  @Test
  public void testRoadStateTransCovLearning1()
      throws TimeOrderException {
    final int iterations = 1000;
    final InferredPath startPath =
        TrackingTestUtils.makeTmpPath(this.graph, false,
            new Coordinate(-Math.pow(iterations, 2), 0d),
            new Coordinate(0d, 0d),
            new Coordinate(Math.pow(iterations, 2), 0d));
    final Matrix covFactor =
        filter.getCovarianceFactor(true);
    final Matrix covar =
        covFactor.times(
            MatrixFactory.getDefault().createDiagonal(
                this.vehicleStateInitialParams
                    .getOnRoadStateCov())).times(
            covFactor.transpose());
    final MultivariateGaussian startState =
        new AdjMultivariateGaussian(
            VectorFactory.getDefault()
                .copyArray(
                    new double[] { 0d,
                        2d }), covar);

    simplePathTest(startPath, startState, iterations);
  }

  /**
   * Slightly jagged path from neg. to pos.
   * @throws TimeOrderException
   */
  @Test(enabled=false)
  public void testRoadStateTransCovLearning2()
      throws TimeOrderException {
    final int iterations = 10000;
    final InferredPath startPath =
        TrackingTestUtils.makeTmpPath(this.graph, false,
            new Coordinate(-3000, -100d),
            new Coordinate(-2000, -100d),
            new Coordinate(-1000, 100d),
            new Coordinate(-2000, 100d),
            new Coordinate(-1000, 0d),
            new Coordinate(0d, 0d),
            new Coordinate(-1000, 0d),
            new Coordinate(-2000, 100d),
            new Coordinate(-1000, 100d),
            new Coordinate(-2000, -100d),
            new Coordinate(-3000, -100d));
    final Matrix covFactor =
        filter.getCovarianceFactor(true);
    final Matrix covar =
        covFactor.times(
            MatrixFactory.getDefault().createDiagonal(
                this.vehicleStateInitialParams
                    .getOnRoadStateCov())).times(
            covFactor.transpose());
    final MultivariateGaussian startState =
        new AdjMultivariateGaussian(
            VectorFactory.getDefault()
                .copyArray(
                    new double[] { 4300d, 0d }), covar);

    simplePathTest(startPath, startState, iterations);
  }

}
