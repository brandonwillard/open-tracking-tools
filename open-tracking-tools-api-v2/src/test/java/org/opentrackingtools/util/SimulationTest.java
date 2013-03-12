package org.opentrackingtools.util;

import org.testng.annotations.BeforeTest;
import org.testng.annotations.DataProvider;
import org.testng.annotations.Test;
import org.testng.AssertJUnit;
import org.testng.internal.junit.ArrayAsserts;
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
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.graph.GenericJTSGraph;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.util.Simulation.SimulationParameters;

import com.vividsolutions.jts.geom.Coordinate;

public class SimulationTest {
  
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
        }
        , {
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
    
    SimulationParameters simParams = new SimulationParameters(
        startCoord, new Date(0l), duration, vehicleStateInitialParams.getInitialObsFreq(), 
        false, true, vehicleStateInitialParams);
    
    sim = new Simulation("test-sim", graph, simParams, 
        vehicleStateInitialParams);
    
    long time = sim.getSimParameters().getStartTime().getTime();
    
    VehicleState<GpsObservation> vehicleState = sim.computeInitialState();
    
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
        System.out.println("Outside of projection!  Flipped state velocities");
      }
      
    } while (time < sim.getSimParameters().getEndTime().getTime());
    
    AssertJUnit.assertTrue(movementSS.getCount() > 0.95d * approxRuns );

    System.out.println(obsErrorSS.getMean());
    System.out.println(movementSS.getMean());
    System.out.println(transitionsSS.getMean());
    
    
  }


  private void updateStats(VehicleState<GpsObservation> vehicleState,
    SufficientStatistic obsErrorSS, SufficientStatistic movementSS,
    SufficientStatistic transitionsSS, boolean generalizeMoveDiff) {
          
    final Vector obsError = vehicleState.getObservation().getProjectedPoint().
        minus(vehicleState.getMeanLocation());
    obsErrorSS.update(obsError);
    
    System.out.println("obsError=" + obsErrorSS.getMean());
    
    final VehicleState<GpsObservation> parentState = vehicleState.getParentState();
    if (parentState != null) {
      final PathState sampledPathState = vehicleState.getPathStateParam().getValue();
      final PathState parentPathState = parentState.getPathStateParam().getValue();
      final PathState predictedPathState = parentPathState 
          .convertToPath(sampledPathState.getPath());
      
      final Vector movementDiff = sampledPathState.minus(predictedPathState);
           
      if (generalizeMoveDiff && movementDiff.getDimensionality() == 4) {
        final Vector movementDiffAvg = avgTransform.times(movementDiff);
        movementSS.update(movementDiffAvg);
      } else {
        movementSS.update(movementDiff);
      }
      System.out.println("movementMean=" + movementSS.getMean());
      
      Vector transType = OnOffEdgeTransDistribution.getTransitionType(
          parentPathState.getEdge().getInferenceGraphEdge(), 
          sampledPathState.getEdge().getInferenceGraphEdge());
      
      if (parentPathState.isOnRoad()) {
        transType = transType.stack(MotionStateEstimatorPredictor.zeros2D);
      } else {
        transType = MotionStateEstimatorPredictor.zeros2D.stack(transType);
      }
      
      transitionsSS.update(transType);
      System.out.println("transitionsMean=" + transitionsSS.getMean());
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
      
      ArrayAsserts.assertArrayEquals(movementZeroArray,
        movementSS.getMean().toArray(), 
        10 * Math.sqrt(vehicleState.getOnRoadModelCovarianceParam().getValue()
            .normFrobenius()));
      
      for (VectorEntry entry : vehicleState.getPathStateParam().getValue()) {
        AssertJUnit.assertTrue(entry.getValue() >= 0);
      }
    }
  }

  private VehicleState<GpsObservation> resetState(VehicleState<GpsObservation> vehicleState) {
    return sim.computeInitialState();
  }
  
}
