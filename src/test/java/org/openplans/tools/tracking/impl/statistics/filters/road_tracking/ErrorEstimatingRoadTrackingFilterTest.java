package org.openplans.tools.tracking.impl.statistics.filters.road_tracking;

import static org.junit.Assert.*;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.stub;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.Date;
import java.util.Random;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.bayesian.BayesianCredibleInterval;
import gov.sandia.cognition.statistics.distribution.InverseWishartDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import org.junit.Before;
import org.junit.Test;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.TimeOrderException;
import org.openplans.tools.tracking.impl.TrackingTestUtils;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.graph.paths.PathStateBelief;
import org.openplans.tools.tracking.impl.statistics.StatisticsUtil;
import org.openplans.tools.tracking.impl.statistics.filters.particle_learning.VehicleTrackingPLFilter;
import org.openplans.tools.tracking.impl.util.GeoUtils;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.common.geometry.GeometryUtils;
import org.opentripplanner.routing.edgetype.PlainStreetEdge;
import org.opentripplanner.routing.vertextype.StreetVertex;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineString;

public class ErrorEstimatingRoadTrackingFilterTest {

  private VehicleStateInitialParameters vehicleStateInitialParams;
  private ErrorEstimatingRoadTrackingFilter filter;
  private OtpGraph graph;

  @Before
  public void setUp() throws Exception {
    
    vehicleStateInitialParams = new VehicleStateInitialParameters(
        VectorFactory.getDefault().createVector2D(100d, 100d), 20,
        VectorFactory.getDefault().createVector1D(6.25e-4), 30,
        VectorFactory.getDefault().createVector2D(6.25e-4, 6.25e-4), 30,
        VectorFactory.getDefault().createVector2D(5d, 95d),
        VectorFactory.getDefault().createVector2D(95d, 5d), 
        VehicleTrackingPLFilter.class.getName(),
        25, 30, 0l);
    
    /*
     * We're using StandardRoadTrackingFilter, but 
     * only the parts with implementations in 
     * AbstractRoadTrackingFilter
     */
    filter = new ErrorEstimatingRoadTrackingFilter(
        vehicleStateInitialParams.getObsCov(),
        vehicleStateInitialParams.getObsCovDof(),
        vehicleStateInitialParams.getOffRoadStateCov(),
        vehicleStateInitialParams.getOffRoadCovDof(),
        vehicleStateInitialParams.getOnRoadStateCov(),
        vehicleStateInitialParams.getOnRoadCovDof(),
        vehicleStateInitialParams.getInitialObsFreq(),
        new Random(1234567890));

    graph = mock(OtpGraph.class);
  }

  
  /**
   * @throws TimeOrderException 
   */
  @Test
  public void testGroundStateTransCovLearning() throws TimeOrderException {
    runErrorTest(false);
  }
  
  /**
   * @throws TimeOrderException 
   */
  @Test
  public void testRoadStateTransCovLearning() throws TimeOrderException {
    runErrorTest(true);
  }
  
  private Vector sampleTransition(Vector state,
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
  
  private void runErrorTest(boolean isOnRoad) {
    final int iterations = 10000;
    InferredPath startPath; 
    InferredPath startPathRev;
    if (isOnRoad) {
      startPath = TrackingTestUtils.makeTmpPath(
          this.graph, false,
          new Coordinate(-Math.pow(iterations, 2), 0d),
          new Coordinate(0d, 0d),
          new Coordinate(Math.pow(iterations, 2), 0d)
          );
      startPathRev = TrackingTestUtils.makeTmpPath(
          this.graph, false,
          new Coordinate(Math.pow(iterations, 2), 0d),
          new Coordinate(0d, 0d),
          new Coordinate(-Math.pow(iterations, 2), 0d)
          );
    } else {
      startPath = InferredPath.getEmptyPath();
      startPathRev = null;
    }
    
    final Matrix covFactor = 
        filter.getCovarianceFactor(isOnRoad);
    
    final Matrix covar = covFactor.times(
        MatrixFactory.getDefault().createDiagonal(
            isOnRoad ? this.vehicleStateInitialParams.getOnRoadStateCov()
                : this.vehicleStateInitialParams.getOffRoadStateCov()
        )).times(covFactor.transpose());
    
    MultivariateGaussian currentStateVec;
    if (isOnRoad) {
      currentStateVec = new MultivariateGaussian(
          VectorFactory.getDefault().copyArray(
             new double[] {iterations * 40d, 1d}), covar);
    } else {
      currentStateVec = new MultivariateGaussian(
          VectorFactory.getDefault().copyArray(
             new double[] {0d, 1d, 0d, 1d}), 
             filter.getOffRoadStateTransCovar());
    }
    
    PathStateBelief currentState = PathStateBelief.
        getPathStateBelief(startPath, currentStateVec);
    
    
    final Random rng = new Random(987654321);
    
    /*
     * Produce a true state and observation.
     */
    
    final Matrix trueStateCov;
    if (isOnRoad) {
      trueStateCov = filter.getOnRoadStateVariancePrior()
          .getMean().clone();
    } else {
      trueStateCov = filter.getOffRoadStateVariancePrior()
          .getMean().clone();
    }
    
    final Matrix trueObsCov = 
        MatrixFactory.getDiagonalDefault().createDiagonal(
        this.vehicleStateInitialParams.getObsCov());
    
    Observation obs = mock(Observation.class);
    stub(obs.getProjectedPoint()).toReturn(
        AbstractRoadTrackingFilter.getOg().times(
        currentState.getGroundState()));
    final Matrix obsCovarChol =
            StatisticsUtil.rootOfSemiDefinite(trueObsCov);
        
    VehicleState state = mock(VehicleState.class);
    stub(state.getBelief()).toReturn(currentState);
    
    PathStateBelief trueState = currentState.clone();
    
    final NumberFormat formatter = new DecimalFormat( "0.000E0" ); 
     
    MultivariateGaussian.SufficientStatistic samplesSS = 
        new MultivariateGaussian.SufficientStatistic();
    MultivariateGaussian.SufficientStatistic residualsSS = 
        new MultivariateGaussian.SufficientStatistic();
    for (int i = 0; i < iterations; i++) {
      System.out.println("i=" + i);
      System.out.println("\tobs=" + obs.getProjectedPoint().toString(formatter));
      System.out.println("\ttrueState=" + trueState.getRawState().toString(formatter));
//      System.out.println("\ttrueStateCov=" 
//        + trueState.getCovariance().convertToVector().toString(formatter));

      /*
       * Perform Kalman steps
       */
      final InferredPath newPath; 
      if (isOnRoad) {
        final InferredEdge presentEdge = currentState.getEdge().getInferredEdge();
        final InferredEdge startEdge = 
            startPath.getEdges().get(0).getInferredEdge();
        
        newPath = presentEdge.getGeometry()
            .equalsTopo(startEdge.getGeometry()) ? startPath : startPathRev;
      } else {
        newPath = startPath;
      }
        
      final PathStateBelief predictedState = 
          filter.predict(currentState, newPath);
      final PathStateBelief updatedState = 
          filter.measure(predictedState, obs.getProjectedPoint(), 
            predictedState.getEdge());
      
      System.out.println("\tcurrentState=" + currentState.getRawState().toString(formatter));
      System.out.println("\tcurrentStateCov=" 
        + currentState.getCovariance().convertToVector().toString(formatter));
      
      final InverseWishartDistribution priorQ = isOnRoad ? 
          filter.getOnRoadStateVariancePrior().clone()
          : filter.getOffRoadStateVariancePrior().clone();    
      
      final Matrix A = isOnRoad ? filter.getRoadModel().getA()
          : filter.getGroundModel().getA();
      
      System.out.println("\tQMean=" + 
          priorQ.getMean().convertToVector().toString(formatter));
      
      System.out.println("\tQ=" + (
          isOnRoad ? filter.getQr().convertToVector().toString(formatter)
              : filter.getQg().convertToVector().toString(formatter))
          );
      
      final InverseWishartDistribution priorObs = filter.getObsVariancePrior().clone();
      
      System.out.println("\tobsMean=" + 
          priorObs.getMean().convertToVector().toString(formatter));
      
      /*
       * Update parameters
       */
      filter.update(state, obs, updatedState, predictedState, rng);
      currentState = updatedState;
      
      stub(state.getBelief()).toReturn(updatedState);
      
      final InverseWishartDistribution posteriorObs = filter.getObsVariancePrior();
      
      System.out.println("\tobsMean=" + 
          filter.getObsVariancePrior().getMean().convertToVector().toString(formatter)
          );
      System.out.println("\t\tobsDiff=" + 
          posteriorObs.getMean().minus(priorObs.getMean()).convertToVector().toString(formatter)
          );
      
      final Vector statesDiff = trueState.getRawState().minus(currentState.getRawState());
      residualsSS.update(statesDiff);
      System.out.println("\t\ttrueStateDiff=" + 
            statesDiff.convertToVector().toString(formatter)
          );
      
      final InverseWishartDistribution posteriorQ = isOnRoad ? filter.getOnRoadStateVariancePrior()
          : filter.getOffRoadStateVariancePrior();
      
      System.out.println("\tQMean=" + 
          posteriorQ.getMean().convertToVector().toString(formatter));
      
       System.out.println("\t\tQMeanDiff=" + 
          posteriorQ.getMean().minus(priorQ.getMean()).convertToVector().toString(formatter));         
       
      System.out.println("\tQ=" + (
          isOnRoad ? filter.getQr().convertToVector().toString(formatter)
              : filter.getQg().convertToVector().toString(formatter))
          );
      if (filter.getCurrentStateSample() != null) {
        System.out.println("\tprevSampleProj=" + 
              A.times(filter.getPrevStateSample().getRawState()).toString(formatter));
        System.out.println("\tcurrSample    =" + 
              filter.getCurrentStateSample().getRawState().toString(formatter));
        final Vector sampleDiff = filter.getCurrentStateSample().getRawState().minus(
                  A.times(filter.getPrevStateSample().getRawState()));
        samplesSS.update(sampleDiff);
        System.out.println("\t\tcurrSampleDiff =" + sampleDiff.toString(formatter));
      }

      
      /*
       * Update true state
       */
      Vector newStateMean = A.times(
              trueState.getRawStateBelief().getMean());
      newStateMean = this.sampleTransition(newStateMean,
          filter, trueStateCov, rng);
      trueState = PathStateBelief.
        getPathStateBelief(startPath, 
            new MultivariateGaussian(newStateMean,
                covar));
      stub(obs.getProjectedPoint()).toReturn(
          MultivariateGaussian.sample(
          AbstractRoadTrackingFilter.getOg().times(
            trueState.getGroundState()),
            obsCovarChol, rng)
          );
    } 
    
    System.out.println("samplesMean=" + 
          samplesSS.getMean().convertToVector().toString(formatter)
        );
    System.out.println("samplesCov=" + 
          samplesSS.getCovariance().toString(formatter)
        );
    
    System.out.println("residualMean=" + 
          residualsSS.getMean().convertToVector().toString(formatter)
        );
    System.out.println("residualCov=" + 
          residualsSS.getCovariance().toString(formatter)
        );
    
    final Matrix obsMean = filter.getObsVariancePrior().getMean();
    assertTrue(obsMean.minus(trueObsCov).normFrobenius()/obsMean.normFrobenius() <= 0.3d);
    
    final Matrix stateTransMean = isOnRoad ? filter.getOnRoadStateVariancePrior().getMean()
            : filter.getOffRoadStateVariancePrior().getMean();
    final Matrix stateTransDiff = stateTransMean.minus(trueStateCov);
    
    assertTrue( stateTransDiff.normFrobenius()/stateTransMean.normFrobenius() <= 0.4d);

  }

}
