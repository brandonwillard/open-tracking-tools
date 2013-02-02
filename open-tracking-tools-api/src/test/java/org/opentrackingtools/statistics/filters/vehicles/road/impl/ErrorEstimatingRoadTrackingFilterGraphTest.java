package org.opentrackingtools.statistics.filters.vehicles.road.impl;

import org.testng.annotations.BeforeTest;
import org.testng.annotations.DataProvider;
import org.testng.annotations.Test;
import org.testng.AssertJUnit;
import org.testng.annotations.Test;
import org.testng.asserts.Assertion;
import org.testng.AssertJUnit;
import org.geotools.data.simple.SimpleFeatureSource;
import org.geotools.factory.FactoryRegistryException;
import org.geotools.feature.FeatureIterator;
import org.geotools.geometry.jts.ReferencedEnvelope;
import org.geotools.grid.Lines;
import org.geotools.grid.ortholine.LineOrientation;
import org.geotools.grid.ortholine.OrthoLineDef;
import org.geotools.referencing.CRS;
import org.geotools.referencing.operation.projection.ProjectionException;

import java.io.IOException;
import java.util.Arrays;
import java.util.Date;
import java.util.List;



import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian.SufficientStatistic;

import org.opengis.feature.Feature;
import org.opengis.referencing.FactoryException;
import org.opengis.referencing.NoSuchAuthorityCodeException;
import org.opengis.referencing.crs.CoordinateReferenceSystem;
import org.opengis.referencing.operation.NoninvertibleTransformException;
import org.opengis.referencing.operation.TransformException;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.impl.GenericJTSGraph;
import org.opentrackingtools.graph.paths.impl.TrackingTestUtils;
import org.opentrackingtools.impl.Simulation;
import org.opentrackingtools.impl.Simulation.SimulationParameters;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.distributions.impl.OnOffEdgeTransDirMulti;
import org.opentrackingtools.statistics.filters.vehicles.impl.VehicleTrackingBootstrapFilter;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.impl.VTErrorEstimatingPLFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.util.GeoUtils;

import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.MultiLineString;

public class ErrorEstimatingRoadTrackingFilterGraphTest {
  
  private InferenceGraph graph;
  private Coordinate startCoord;
  private Matrix avgTransform;
  private Simulation sim;
  private double[] movementZeroArray;
  private double[] obsErrorZeroArray;
  private static final double[] sixteenZeros = new double[] {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  private static final double[] fourZeros = new double[] {0, 0, 0, 0};
  private static final double[] twoZeros = new double[] {0, 0};

  @BeforeTest
  public void setUp() throws NoSuchAuthorityCodeException, FactoryRegistryException, FactoryException, IOException {
    
    startCoord = new Coordinate(40.7549, -73.97749);
    
    List<LineString> edges = TrackingTestUtils.createGridGraph(startCoord);
    graph = new GenericJTSGraph(edges);
    
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
        new VehicleStateInitialParameters(VectorFactory
            .getDefault().createVector2D(100d, 100d), 20,
            VectorFactory.getDefault().createVector1D(
                6.25e-4), 30, VectorFactory.getDefault()
                .createVector2D(6.25e-4, 6.25e-4), 20,
            VectorFactory.getDefault().createVector2D(1d,
                Double.MAX_VALUE), VectorFactory.getDefault()
                .createVector2D(Double.MAX_VALUE, 1d),
            VehicleTrackingBootstrapFilter.class.getName(), 25,
            15, 2159585l),
        Boolean.FALSE,
            66000
        },
        {
          /*
           * Ground only
           */
        new VehicleStateInitialParameters(VectorFactory
            .getDefault().createVector2D(100d, 100d), 20,
            VectorFactory.getDefault().createVector1D(
                6.25e-4), 30, VectorFactory.getDefault()
                .createVector2D(6.25e-5, 6.25e-5), 20,
            VectorFactory.getDefault().createVector2D(
                Double.MAX_VALUE, 1d), 
            VectorFactory.getDefault().createVector2D(
                1d, Double.MAX_VALUE),
            VehicleTrackingBootstrapFilter.class.getName(), 25,
            10, 215955l),
        Boolean.FALSE,
            66000
        }, 
        {
          /*
           * Mixed 
           */
        new VehicleStateInitialParameters(VectorFactory
            .getDefault().createVector2D(100d, 100d), 20,
            VectorFactory.getDefault().createVector1D(
                6.25e-4), 30, VectorFactory.getDefault()
                .createVector2D(6.25e-4, 6.25e-4), 20,
            VectorFactory.getDefault().createVector2D(
                1d, 1d), 
            VectorFactory.getDefault().createVector2D(
                1d, 1d),
            VehicleTrackingBootstrapFilter.class.getName(), 25,
            15, 21595857l), 
            Boolean.TRUE,
            46000
        }
    };
  }
  
  @Test(dataProvider="initialStateData")
  public void runSimulation(VehicleStateInitialParameters vehicleStateInitialParams,
    boolean generalizeMoveDiff, long duration) throws NoninvertibleTransformException, TransformException {
    
    SimulationParameters simParams = new SimulationParameters(
        startCoord, new Date(0l), duration, 15, false, vehicleStateInitialParams);
    
    sim = new Simulation("test-sim", graph, simParams, 
        vehicleStateInitialParams);
    
    long time = sim.getSimParameters().getStartTime().getTime();
    
    VehicleState trueVehicleState = sim.computeInitialState();
    
    VTErrorEstimatingPLFilter filter = 
      new VTErrorEstimatingPLFilter(trueVehicleState.getObservation(),
          graph, vehicleStateInitialParams, true);
    
    DataDistribution<VehicleState> vehicleStateDist = 
        filter.createInitialLearnedObject();
    
    final MultivariateGaussian.SufficientStatistic obsErrorSS =
        new MultivariateGaussian.SufficientStatistic();
    final MultivariateGaussian.SufficientStatistic obsCovErrorSS =
        new MultivariateGaussian.SufficientStatistic();
    final MultivariateGaussian.SufficientStatistic onRoadCovErrorSS =
        new MultivariateGaussian.SufficientStatistic();
    final MultivariateGaussian.SufficientStatistic offRoadCovErrorSS =
        new MultivariateGaussian.SufficientStatistic();
    final MultivariateGaussian.SufficientStatistic transitionsSS =
        new MultivariateGaussian.SufficientStatistic();
    
    final long approxRuns = sim.getSimParameters().getDuration()/
        sim.getSimParameters().getFrequency();
    
    updateStats(vehicleStateDist, trueVehicleState, obsErrorSS, obsCovErrorSS, 
        onRoadCovErrorSS, offRoadCovErrorSS, transitionsSS, generalizeMoveDiff);
    
    do {
      try {
        trueVehicleState = sim.stepSimulation(trueVehicleState);
        
        filter.update(vehicleStateDist, trueVehicleState.getObservation());
        
        updateStats(vehicleStateDist, trueVehicleState, obsErrorSS, obsCovErrorSS, 
            onRoadCovErrorSS, offRoadCovErrorSS, transitionsSS, generalizeMoveDiff);
        
        time = trueVehicleState.getObservation().getTimestamp().getTime();
      } catch (ProjectionException ex) {
        trueVehicleState = resetState(trueVehicleState);
//        if (vehicleState.getParentState() != null)
//          vehicleState.setParentState(resetState(vehicleState.getParentState()));
        System.out.println("Outside of projection!  Flipped state velocities");
      }
      
    } while (time < sim.getSimParameters().getEndTime().getTime());
    
    AssertJUnit.assertTrue(transitionsSS.getCount() > 0.95d * approxRuns );
  }


  private void updateStats(DataDistribution<VehicleState> vehicleStateDist, 
    VehicleState trueVehicleState,
    SufficientStatistic obsErrorSS, 
    SufficientStatistic obsCovErrorSS,
    SufficientStatistic onRoadCovErrorSS,
    SufficientStatistic offRoadCovErrorSS,
    SufficientStatistic transitionsSS, boolean generalizeMoveDiff) {
    
    
    SufficientStatistic obsErrorStat = 
        new MultivariateGaussian.SufficientStatistic();
    SufficientStatistic obsCovErrorStat = 
        new MultivariateGaussian.SufficientStatistic();
    SufficientStatistic onRoadCovErrorStat = 
        new MultivariateGaussian.SufficientStatistic();
    SufficientStatistic offRoadCovErrorStat = 
        new MultivariateGaussian.SufficientStatistic();
    SufficientStatistic transitionsErrorStat = 
        new MultivariateGaussian.SufficientStatistic();
        
    for (VehicleState state : vehicleStateDist.getDomain()) {
      
      AssertJUnit.assertEquals(trueVehicleState.getObservation(),
          state.getObservation());
      
      final Vector obsError = trueVehicleState.getObservation().getProjectedPoint().
          minus(state.getMeanLocation());
      obsErrorStat.update(obsError);
      
      final Matrix obsCovMean = ((ErrorEstimatingRoadTrackingFilter) state.getMovementFilter())
        .getObsVariancePrior().getMean();
      final Matrix obsCovError = obsCovMean.minus(
          trueVehicleState.getMovementFilter().getObsCovar());
      obsCovErrorStat.update(obsCovError.convertToVector());
      
      final Matrix onRoadCovMean = ((ErrorEstimatingRoadTrackingFilter) state.getMovementFilter())
        .getOnRoadStateVariancePrior().getMean();
      final Matrix onRoadCovFactor = state.getMovementFilter().getCovarianceFactor(true);
      final Matrix onRoadCovError = onRoadCovFactor.times(onRoadCovMean)
          .times(onRoadCovFactor.transpose()).minus(
          trueVehicleState.getMovementFilter().getOnRoadStateTransCovar());
      onRoadCovErrorStat.update(onRoadCovError.convertToVector());
      
      final Matrix offRoadCovMean = ((ErrorEstimatingRoadTrackingFilter) state.getMovementFilter())
        .getOffRoadStateVariancePrior().getMean();
      final Matrix offRoadCovFactor = state.getMovementFilter().getCovarianceFactor(false);
      final Matrix offRoadCovError = offRoadCovFactor.times(offRoadCovMean)
          .times(offRoadCovFactor.transpose()).minus(
          trueVehicleState.getMovementFilter().getOffRoadStateTransCovar());
      offRoadCovErrorStat.update(offRoadCovError.convertToVector());
      
      final VehicleState parentState = state.getParentState();
      if (parentState != null) {
        
        Vector transType = OnOffEdgeTransDirMulti.getTransitionType(
            parentState.getBelief().getEdge().getInferredEdge(), 
            state.getBelief().getEdge().getInferredEdge());
        
        if (parentState.getBelief().isOnRoad()) {
          transType = transType.stack(AbstractRoadTrackingFilter.zeros2D);
        } else {
          transType = AbstractRoadTrackingFilter.zeros2D.stack(transType);
        }
        
        final Vector trueTransProbs = trueVehicleState.getEdgeTransitionDist()
            .getEdgeMotionTransPrior().getMean().stack(
                trueVehicleState.getEdgeTransitionDist().
                getFreeMotionTransPrior().getMean());
        
        transitionsErrorStat.update(
            transType.minus(trueTransProbs));
      }
    }
    
    obsErrorSS.update(obsErrorStat.getMean());
    obsCovErrorSS.update(obsCovErrorStat.getMean());
    onRoadCovErrorSS.update(onRoadCovErrorStat.getMean());
    offRoadCovErrorSS.update(offRoadCovErrorStat.getMean());
    if (transitionsErrorStat.getMean() != null)
      transitionsSS.update(transitionsErrorStat.getMean());
      
    System.out.println("obsError=" + obsErrorSS.getMean());
    System.out.println("obsCovError=" + obsCovErrorSS.getMean());
    System.out.println("onRoadCovError=" + onRoadCovErrorSS.getMean());
    System.out.println("offRoadCovError=" + offRoadCovErrorSS.getMean());
    System.out.println("transitionsMean=" + transitionsSS.getMean());
    
    final long approxRuns = sim.getSimParameters().getDuration()/
        sim.getSimParameters().getFrequency();
    if (obsErrorSS.getCount() > Math.min(approxRuns/16, 25)) {
      
      if (obsErrorZeroArray == null)
        obsErrorZeroArray = VectorFactory.getDefault()
          .createVector(obsErrorSS.getMean().getDimensionality())
          .toArray();
      
      AssertJUnit.assertArrayEquals(
          obsErrorSS.getMean().getDimensionality() == 4 ?
              fourZeros : twoZeros,
        obsErrorSS.getMean().toArray(), 5);
    
//      AssertJUnit.assertArrayEquals(fourZeros,
//        obsCovErrorSS.getMean().toArray(), 1);
//      
//      AssertJUnit.assertArrayEquals(fourZeros,
//        onRoadCovErrorSS.getMean().toArray(), 0.01);
//      
//      AssertJUnit.assertArrayEquals(sixteenZeros,
//        offRoadCovErrorSS.getMean().toArray(), 0.01);
    }
  }


  private VehicleState resetState(VehicleState vehicleState) {
    
    return sim.computeInitialState();
//    final MultivariateGaussian beliefDist = 
//          vehicleState.getBelief().getGlobalStateBelief().clone();
//    if (vehicleState.getBelief().isOnRoad()) {
//      beliefDist.getMean().setElement(1, -beliefDist.getMean().getElement(1));
//    } else {
//      beliefDist.getMean().setElement(1, -beliefDist.getMean().getElement(1));
//      beliefDist.getMean().setElement(3, -beliefDist.getMean().getElement(3));
//    }
//    PathStateBelief belief = 
//          vehicleState.getBelief().getPath().getStateBeliefOnPath(beliefDist);
//    return new VehicleState(graph, 
//        vehicleState.getObservation(), 
//        vehicleState.getMovementFilter(), 
//        belief, vehicleState.edgeTransitionDist, 
//        vehicleState.getParentState());
  }
  
}
