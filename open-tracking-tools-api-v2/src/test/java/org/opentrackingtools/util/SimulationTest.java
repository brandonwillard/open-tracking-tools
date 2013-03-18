package org.opentrackingtools.util;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.testng.annotations.BeforeTest;
import org.testng.annotations.DataProvider;
import org.testng.annotations.Test;
import org.testng.AssertJUnit;
import org.testng.internal.junit.ArrayAsserts;
import org.apache.log4j.BasicConfigurator;
import org.geotools.factory.FactoryRegistryException;
import org.geotools.referencing.operation.projection.ProjectionException;

import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.util.Date;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorEntry;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian.SufficientStatistic;

import org.opengis.referencing.FactoryException;
import org.opengis.referencing.NoSuchAuthorityCodeException;
import org.opengis.referencing.operation.NoninvertibleTransformException;
import org.opengis.referencing.operation.TransformException;
import org.opentrackingtools.VehicleStateInitialParameters;
import org.opentrackingtools.distributions.OnOffEdgeTransDistribution;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.distributions.TruncatedRoadGaussian;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.graph.GenericJTSGraph;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.util.Simulation.SimulationParameters;

import com.vividsolutions.jts.geom.Coordinate;

public class SimulationTest {
  
  private static final Logger _log = LoggerFactory
      .getLogger(SimulationTest.class);
  
  private InferenceGraph graph;
  private Coordinate startCoord;
  private Matrix avgTransform;
  private double[] movementZeroArray;
  private double[] obsErrorZeroArray;
  private Simulation sim;

  @BeforeTest
  public void setUp() throws NoSuchAuthorityCodeException, FactoryRegistryException, FactoryException, IOException {
    
    startCoord = new Coordinate(40.7549, -73.97749);
    
    graph = new GenericJTSGraph(TestUtils.createGridGraph(startCoord), true);
    
    avgTransform = MatrixFactory.getDefault().copyArray(
        new double[][] {
            {1, 0, 1, 0},
            {0, 1, 0, 1}
        }).scale(1d/2d);
  }
  
  
  @DataProvider
  private static final Object[][] initialStateData() {
    return new Object[][] {
        {
          /*
           * Road only
           */
        new VehicleStateInitialParameters(
            null,
            VectorFactory.getDefault().createVector2D(100d, 100d), 20,
            VectorFactory.getDefault().createVector1D(
                6.25e-4), 30, VectorFactory.getDefault()
                .createVector2D(6.25e-4, 6.25e-4), 20,
            VectorFactory.getDefault().createVector2D(1d,
                Double.MAX_VALUE), VectorFactory.getDefault()
                .createVector2D(Double.MAX_VALUE, 1d),
            25, 15, 2159585l),
        Boolean.FALSE,
            126000
        },
        {
          /*
           * Ground only
           */
        new VehicleStateInitialParameters(
            null,
            VectorFactory.getDefault().createVector2D(100d, 100d), 20,
            VectorFactory.getDefault().createVector1D(
                6.25e-4), 30, VectorFactory.getDefault()
                .createVector2D(6.25e-4, 6.25e-4), 20,
            VectorFactory.getDefault().createVector2D(
                Double.MAX_VALUE, 1d), 
            VectorFactory.getDefault().createVector2D(
                1d, Double.MAX_VALUE),
            25, 10, 215955l),
        Boolean.FALSE,
            126000
        },
        {
          /*
           * Mixed 
           */
        new VehicleStateInitialParameters(
            null,
            VectorFactory.getDefault().createVector2D(100d, 100d), 20,
            VectorFactory.getDefault().createVector1D(
                6.25e-4), 30, VectorFactory.getDefault()
                .createVector2D(6.25e-4, 6.25e-4), 20,
            VectorFactory.getDefault().createVector2D(
                1d, 1d), 
            VectorFactory.getDefault().createVector2D(
                1d, 1d),
            25, 15, 21595857l), 
            Boolean.TRUE,
            46000
        }
    };
  }
  @Test(dataProvider="initialStateData")
  public void runSimulation(VehicleStateInitialParameters vehicleStateInitialParams,
    boolean generalizeMoveDiff, long duration) throws NoninvertibleTransformException, TransformException, SecurityException, IllegalArgumentException, ClassNotFoundException, NoSuchMethodException, InstantiationException, IllegalAccessException, InvocationTargetException {
    BasicConfigurator.configure();
    
    SimulationParameters simParams = new SimulationParameters(
        startCoord, new Date(0l), duration, vehicleStateInitialParams.getInitialObsFreq(), 
        false, true, vehicleStateInitialParams);
    
    sim = new Simulation("test-sim", graph, simParams, 
        vehicleStateInitialParams);
    
    long time = sim.getSimParameters().getStartTime().getTime();
    
    VehicleStateDistribution<GpsObservation> vehicleState = sim.computeInitialState();
    
    final MultivariateGaussian.SufficientStatistic obsErrorSS =
        new MultivariateGaussian.SufficientStatistic();
    final MultivariateGaussian.SufficientStatistic movementSS =
        new MultivariateGaussian.SufficientStatistic();
    final MultivariateGaussian.SufficientStatistic transitionsSS =
        new MultivariateGaussian.SufficientStatistic();
    
    obsErrorZeroArray = null;
    movementZeroArray = null;
    
    final long approxRuns = sim.getSimParameters().getDuration()/
        Math.round(sim.getSimParameters().getFrequency());
    
    updateStats(vehicleState, obsErrorSS, movementSS, transitionsSS, 
        generalizeMoveDiff);
    
    do {
      try {
        vehicleState = sim.stepSimulation(vehicleState);
        updateStats(vehicleState, obsErrorSS, movementSS, transitionsSS, 
            generalizeMoveDiff);
        time = vehicleState.getObservation().getTimestamp().getTime();
      } catch (ProjectionException ex) {
        vehicleState = resetState(vehicleState);
//        if (vehicleState.getParentState() != null)
//          vehicleState.setParentState(resetState(vehicleState.getParentState()));
        _log.warn("Outside of projection!  Flipped state velocities");
      }
      
    } while (time < sim.getSimParameters().getEndTime().getTime());
    
    AssertJUnit.assertTrue(movementSS.getCount() > 0.95d * approxRuns );

    _log.info(obsErrorSS.getMean().toString());
    _log.info(movementSS.getMean().toString());
    _log.info(transitionsSS.getMean().toString());
    
    
  }


  private void updateStats(VehicleStateDistribution<GpsObservation> vehicleState,
    SufficientStatistic obsErrorSS, SufficientStatistic movementSS,
    SufficientStatistic transitionsSS, boolean generalizeMoveDiff) {
          
    final Vector obsError = vehicleState.getObservation().getProjectedPoint().
        minus(vehicleState.getMeanLocation());
    obsErrorSS.update(obsError);
    
    _log.info("obsError=" + obsErrorSS.getMean());
    
    MotionStateEstimatorPredictor motionStateEstimator = new MotionStateEstimatorPredictor(vehicleState, null, 
        this.sim.getSimParameters().getFrequency());
    
    final VehicleStateDistribution<GpsObservation> parentState = vehicleState.getParentState();
    if (parentState != null) {
      final PathState sampledPathState = vehicleState.getPathStateParam().getValue();
      final PathState parentPathState = parentState.getPathStateParam().getValue();
      final MultivariateGaussian priorState;
      final MultivariateGaussian predictedMotionStateDist;
      if (sampledPathState.getPath().isNullPath() && !parentPathState.getPath().isNullPath()) {
        priorState = new 
          TruncatedRoadGaussian(parentPathState.getGroundState(),
              MatrixFactory.getDefault().createMatrix(4, 4),
              Double.MAX_VALUE, 0d);
        final Vector expectedError = this.sim.getUpdater().getSampledTransitionError();
        predictedMotionStateDist = 
          motionStateEstimator.createPredictiveDistribution(priorState);
            
        predictedMotionStateDist.setMean(predictedMotionStateDist.getMean().plus(expectedError));
      } else if (!sampledPathState.getPath().isNullPath() && parentPathState.getPath().isNullPath()) {
        priorState = new 
          TruncatedRoadGaussian(
              parentPathState.getEdgeState(), 
              MatrixFactory.getDefault().createMatrix(4, 4),
              Double.MAX_VALUE, 0d);
        predictedMotionStateDist = 
          motionStateEstimator.createPredictiveDistribution(priorState);
        predictedMotionStateDist.getMean().plusEquals(this.sim.getUpdater().getSampledTransitionError());
        PathUtils.convertToRoadBelief(predictedMotionStateDist,sampledPathState.getPath(), 
            sampledPathState.getEdge(), true);
      } else {
        priorState = new 
          TruncatedRoadGaussian(
              parentState.getPathStateParam().getValue().getEdgeState(), 
              parentState.getMotionStateParam().getParameterPrior().getCovariance(),
              Double.MAX_VALUE, 0d);
        final Vector expectedError = this.sim.getUpdater().getSampledTransitionError();
        predictedMotionStateDist = 
          motionStateEstimator.createPredictiveDistribution(priorState);
            
        predictedMotionStateDist.setMean(predictedMotionStateDist.getMean().plus(expectedError));
      }
      final PathState predictedPathState = 
          new PathState(sampledPathState.getPath(), 
          predictedMotionStateDist.getMean());
      
      final Vector movementDiff = sampledPathState.minus(predictedPathState);
      AssertJUnit.assertArrayEquals(
          VectorFactory.getDefault().createVector(movementDiff.getDimensionality()).toArray(), 
          movementDiff.toArray(), 1e-5);
           
      if (generalizeMoveDiff && movementDiff.getDimensionality() == 4) {
        final Vector movementDiffAvg = avgTransform.times(movementDiff);
        movementSS.update(movementDiffAvg);
      } else {
        movementSS.update(movementDiff);
      }
      _log.info("movementError=" + movementDiff);
      _log.info("movementMean=" + movementSS.getMean());
      
      Vector transType = OnOffEdgeTransDistribution.getTransitionType(
          parentPathState.getEdge().getInferenceGraphEdge(), 
          sampledPathState.getEdge().getInferenceGraphEdge());
      
      if (parentPathState.isOnRoad()) {
        transType = transType.stack(MotionStateEstimatorPredictor.zeros2D);
      } else {
        transType = MotionStateEstimatorPredictor.zeros2D.stack(transType);
      }
      
      transitionsSS.update(transType);
      _log.info("transitionsMean=" + transitionsSS.getMean());
    }
    
    final long approxRuns = sim.getSimParameters().getDuration()/
        Math.round(sim.getSimParameters().getFrequency());
    if (movementSS.getCount() > 0) {//Math.min(approxRuns/16, 25)) {
      if (obsErrorZeroArray == null)
        obsErrorZeroArray = VectorFactory.getDefault()
          .createVector(obsErrorSS.getMean().getDimensionality())
          .toArray();
      
      ArrayAsserts.assertArrayEquals(obsErrorZeroArray,
        obsErrorSS.getMean().toArray(), 5 * Math.sqrt(
            vehicleState.getObservationCovarianceParam().getValue().normFrobenius()));
    
      if (movementZeroArray == null)
        movementZeroArray = VectorFactory.getDefault()
          .createVector(movementSS.getMean().getDimensionality())
          .toArray();
      
      final Matrix stateCovariance;
      if (vehicleState.getPathStateParam().getValue().isOnRoad()) {
        final Matrix covFactor = motionStateEstimator.getCovarianceFactor(true);
        stateCovariance = covFactor.times(vehicleState.getOnRoadModelCovarianceParam().getValue())
            .times(covFactor.transpose());
      } else {
        final Matrix covFactor = motionStateEstimator.getCovarianceFactor(false);
        stateCovariance = covFactor.times(vehicleState.getOffRoadModelCovarianceParam().getValue())
            .times(covFactor.transpose());
      }
          
      ArrayAsserts.assertArrayEquals(movementZeroArray,
        movementSS.getMean().toArray(), 
        10 * Math.sqrt(stateCovariance.normFrobenius()));
      
      if (vehicleState.getPathStateParam().getValue().isOnRoad()) {
        for (VectorEntry entry : vehicleState.getPathStateParam().getValue()) {
          AssertJUnit.assertTrue(entry.getValue() >= 0);
        }
      }
    }
  }

  private VehicleStateDistribution<GpsObservation> resetState(VehicleStateDistribution<GpsObservation> vehicleState) {
    return sim.computeInitialState();
  }
  
}
