package org.opentrackingtools.statistics.filters.vehicles.road.impl;

import org.testng.annotations.BeforeTest;
import org.testng.annotations.DataProvider;
import org.testng.annotations.Test;
import org.testng.AssertJUnit;
import org.testng.asserts.Assertion;
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
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.Arrays;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Random;



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
import org.opentrackingtools.impl.SimpleObservation;
import org.opentrackingtools.impl.Simulation;
import org.opentrackingtools.impl.Simulation.SimulationParameters;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.distributions.impl.OnOffEdgeTransDirMulti;
import org.opentrackingtools.statistics.filters.vehicles.VehicleTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.impl.VehicleTrackingBootstrapFilter;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.impl.VehicleTrackingPLFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.util.GeoUtils;
import org.opentrackingtools.util.geom.ProjectedCoordinate;
import org.opentrackingtools.GpsObservation;

import com.google.common.base.Objects;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.MultiLineString;

public class RoadTrackingFilterGraphTest {
  
  private InferenceGraph graph;
  private Coordinate startCoord;
  private Matrix avgTransform;
  private Simulation sim;
  private static final double[] sixteenZeros = VectorFactory.getDefault()
      .createVector(16).toArray() ;
  private static final double[] fourZeros = new double[] {0, 0, 0, 0};
  private static final double[] twoZeros = new double[] {0, 0};
  private static final double[] oneZero = new double[] {0};
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
            .getDefault().createVector2D(70d, 70d), 20,
            VectorFactory.getDefault().createVector1D(
                6.25e-4), 30, VectorFactory.getDefault()
                .createVector2D(6.25e-4, 6.25e-4), 20,
            VectorFactory.getDefault().createVector2D(1d,
                Double.MAX_VALUE), VectorFactory.getDefault()
                .createVector2D(Double.MAX_VALUE, 1d),
            VehicleTrackingPLFilter.class.getName(), 
            StandardRoadTrackingFilter.class.getName(), 
            25, 15, 2159585l),
        Boolean.FALSE,
            66000
        }
//        , {
//          /*
//           * Ground only
//           */
//        new VehicleStateInitialParameters(VectorFactory
//            .getDefault().createVector2D(100d, 100d), 20,
//            VectorFactory.getDefault().createVector1D(
//                6.25e-4), 30, VectorFactory.getDefault()
//                .createVector2D(6.25e-5, 6.25e-5), 20,
//            VectorFactory.getDefault().createVector2D(
//                Double.MAX_VALUE, 1d), 
//            VectorFactory.getDefault().createVector2D(
//                1d, Double.MAX_VALUE),
//            null, 25,
//            10, 215955l),
//        Boolean.FALSE,
//            66000
//        }, 
//        {
//          /*
//           * Mixed 
//           */
//        new VehicleStateInitialParameters(VectorFactory
//            .getDefault().createVector2D(100d, 100d), 20,
//            VectorFactory.getDefault().createVector1D(
//                6.25e-4), 30, VectorFactory.getDefault()
//                .createVector2D(6.25e-4, 6.25e-4), 20,
//            VectorFactory.getDefault().createVector2D(
//                1d, 1d), 
//            VectorFactory.getDefault().createVector2D(
//                1d, 1d),
//            null, 25,
//            15, 21595857l), 
//            Boolean.TRUE,
//            46000
//        }
    };
  }
  
  public static class TrueObservation extends SimpleObservation {

    final private VehicleState trueState;

    public TrueObservation(GpsObservation obs, VehicleState trueState) {
      super(obs.getSourceId(), obs.getTimestamp(), 
          obs.getObsCoordsLatLon(), obs.getVelocity(), obs.getHeading(), 
          obs.getAccuracy(), obs.getRecordNumber(), obs.getPreviousObservation(), 
          obs.getObsProjected());
      this.trueState = trueState;
    }

    public VehicleState getTrueState() {
      return trueState;
    }

  }
  
  @Test(dataProvider="initialStateData")
  public void runSimulation(VehicleStateInitialParameters vehicleStateInitialParams,
    boolean generalizeMoveDiff, long duration) throws NoninvertibleTransformException, TransformException, ClassNotFoundException, SecurityException, NoSuchMethodException, IllegalArgumentException, InstantiationException, IllegalAccessException, InvocationTargetException {
    
    SimulationParameters simParams = new SimulationParameters(
        startCoord, new Date(0l), duration, 15, false, false, 
        vehicleStateInitialParams);
    
    sim = new Simulation("test-sim", graph, simParams, 
        vehicleStateInitialParams);
    
    long time = sim.getSimParameters().getStartTime().getTime();
    
    VehicleState trueVehicleState = sim.computeInitialState();
    
    Class<?> filterType = 
        Class.forName(vehicleStateInitialParams.getParticleFilterTypeName());
      
    Constructor<?> ctor = filterType.getConstructor(GpsObservation.class, 
          InferenceGraph.class,
          VehicleStateInitialParameters.class, 
          Boolean.class, Random.class);
      
    Random rng;
    if (vehicleStateInitialParams.getSeed() != 0)
      rng = new Random(vehicleStateInitialParams.getSeed());
    else
      rng = new Random();
    
    VehicleTrackingFilter<GpsObservation, VehicleState> filter = 
        (VehicleTrackingFilter) ctor.newInstance(
          new TrueObservation(trueVehicleState.getObservation(), 
              trueVehicleState), graph, vehicleStateInitialParams, true, rng);
    
//    VTErrorEstimatingPLFilter filter = 
//      new VTErrorEstimatingPLFilter(
//          new TrueObservation(trueVehicleState.getObservation(), 
//              trueVehicleState),
//          graph, vehicleStateInitialParams, true);
//    VehicleTrackingPLFilter filter = 
//      new VehicleTrackingPLFilter(
//          new TrueObservation(trueVehicleState.getObservation(), 
//              trueVehicleState),
//          graph, vehicleStateInitialParams, true);
    
    DataDistribution<VehicleState> vehicleStateDist = 
        filter.createInitialLearnedObject();
    
    final MultivariateGaussian.SufficientStatistic obsErrorSS =
        new MultivariateGaussian.SufficientStatistic();
    final MultivariateGaussian.SufficientStatistic stateErrorSS =
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
    
    updateStats(vehicleStateDist, trueVehicleState, obsErrorSS, stateErrorSS, 
        obsCovErrorSS, onRoadCovErrorSS, offRoadCovErrorSS, transitionsSS, 
        generalizeMoveDiff);
    
    do {
      try {
        trueVehicleState = sim.stepSimulation(trueVehicleState);
        
        filter.update(vehicleStateDist, 
            new TrueObservation(trueVehicleState.getObservation(), 
                trueVehicleState));
        
        updateStats(vehicleStateDist, trueVehicleState, obsErrorSS, stateErrorSS, 
            obsCovErrorSS, onRoadCovErrorSS, offRoadCovErrorSS, transitionsSS, 
            generalizeMoveDiff);
        
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
    SufficientStatistic stateErrorSS, 
    SufficientStatistic obsCovErrorSS,
    SufficientStatistic onRoadCovErrorSS,
    SufficientStatistic offRoadCovErrorSS,
    SufficientStatistic transitionsSS, boolean generalizeMoveDiff) {
    
    
    SufficientStatistic stateMeanStat = 
        new MultivariateGaussian.SufficientStatistic();
    SufficientStatistic obsCovMeanStat = 
        new MultivariateGaussian.SufficientStatistic();
    SufficientStatistic onRoadCovStat = 
        new MultivariateGaussian.SufficientStatistic();
    SufficientStatistic offRoadCovStat = 
        new MultivariateGaussian.SufficientStatistic();
    SufficientStatistic transitionStat = 
        new MultivariateGaussian.SufficientStatistic();
        
    int truePathsFound = 0;
    int numOnTrueEdge = 0;
    boolean hasPriorOnVariances = false;
    
    for (VehicleState state : vehicleStateDist.getDomain()) {
      
      AssertJUnit.assertEquals(
          new TrueObservation(trueVehicleState.getObservation(), null),
          state.getObservation());
      
      if (state.getBelief().getPath().getGeometry() != null
          && state.getBelief().getPath().getGeometry().covers(
            trueVehicleState.getBelief().getPath().getGeometry()))
        truePathsFound++;
      
      if (state.getBelief().getEdge().getInferredEdge().equals(
          trueVehicleState.getBelief().getEdge().getInferredEdge()))
        numOnTrueEdge++;
          
      stateMeanStat.update(state.getBelief().getGroundState());
      if (state.getMovementFilter() instanceof ErrorEstimatingRoadTrackingFilter) {
        hasPriorOnVariances = true;
        final Matrix obsCovMean = ((ErrorEstimatingRoadTrackingFilter) state.getMovementFilter())
          .getObsVariancePrior().getMean();
        obsCovMeanStat.update(obsCovMean.convertToVector());
        final Matrix onRoadCovMean = ((ErrorEstimatingRoadTrackingFilter) state.getMovementFilter())
          .getOnRoadStateVariancePrior().getMean();
        onRoadCovStat.update(onRoadCovMean.convertToVector());
        final Matrix offRoadCovMean = ((ErrorEstimatingRoadTrackingFilter) state.getMovementFilter())
          .getOffRoadStateVariancePrior().getMean();
        offRoadCovStat.update(offRoadCovMean.convertToVector());
      } else {
        obsCovMeanStat.update(state.getMovementFilter().getObsCovar()
            .convertToVector());
        onRoadCovStat.update(state.getMovementFilter().getOnRoadStateTransCovar()
            .convertToVector());
        offRoadCovStat.update(state.getMovementFilter().getOffRoadStateTransCovar()
            .convertToVector());
      }
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
        
        transitionStat.update(transType);
      }
      
    }
      
    System.out.println("truePathsFound=" + truePathsFound);
    System.out.println("numOnTrueEdges=" + numOnTrueEdge);
      
    final Vector obsError = trueVehicleState.getObservation().getProjectedPoint().
        minus(AbstractRoadTrackingFilter.getOg().times(stateMeanStat.getMean()));
    final Vector stateError = trueVehicleState.getBelief().getGroundState().
        minus(stateMeanStat.getMean());
    
    final Vector obsCovError;   
    final Vector onRoadCovError;
    final Vector offRoadCovError;
    if (hasPriorOnVariances) {
      obsCovError = obsCovMeanStat.getMean().minus(
          trueVehicleState.getMovementFilter().getObsCovar().convertToVector());
      onRoadCovError = onRoadCovStat.getMean().minus(
          trueVehicleState.getMovementFilter().getQr().convertToVector());
      offRoadCovError = offRoadCovStat.getMean().minus(
          trueVehicleState.getMovementFilter().getQg().convertToVector());
    } else {
      obsCovError = obsCovMeanStat.getMean().minus(
          trueVehicleState.getMovementFilter().getObsCovar().convertToVector());
      onRoadCovError = onRoadCovStat.getMean().minus(
          trueVehicleState.getMovementFilter().getOnRoadStateTransCovar().convertToVector());
      offRoadCovError = offRoadCovStat.getMean().minus(
          trueVehicleState.getMovementFilter().getOffRoadStateTransCovar().convertToVector());
    }
    
    System.out.println("obsError=" + obsError);
    System.out.println("stateError=" + stateError);
    if (hasPriorOnVariances) {
      System.out.println("obsCovMean=" + obsCovMeanStat.getMean());
      System.out.println("onRoadCovMean=" + onRoadCovStat.getMean());
  //    System.out.println("offRoadCovMean=" + offRoadCovStat.getMean());
    }
    
    if (transitionStat.getMean() != null) {
      final Vector trueTransProbs = trueVehicleState.getEdgeTransitionDist()
          .getEdgeMotionTransPrior().getMean().stack(
              trueVehicleState.getEdgeTransitionDist().
              getFreeMotionTransPrior().getMean());
      
      transitionsSS.update(transitionStat.getMean().minus(trueTransProbs));
//      System.out.println("transitionMean=" + transitionStat.getMean());
    }
    
    obsErrorSS.update(obsError);
    stateErrorSS.update(stateError);
    obsCovErrorSS.update(obsCovError);
    onRoadCovErrorSS.update(onRoadCovError);
    offRoadCovErrorSS.update(offRoadCovError);
      
    if (!hasPriorOnVariances) {
      System.out.println("obsErrorSS=" + obsErrorSS.getMean());
      System.out.println("stateErrorSS=" + stateErrorSS.getMean());
//      System.out.println("obsCovErrorSS=" + obsCovErrorSS.getMean());
//      System.out.println("onRoadCovErrorSS=" + onRoadCovErrorSS.getMean());
//      System.out.println("offRoadCovErrorSS=" + offRoadCovErrorSS.getMean());
//      System.out.println("transitionErrorSS=" + transitionsSS.getMean());
    }
    
    final long approxRuns = sim.getSimParameters().getDuration()/
        sim.getSimParameters().getFrequency();
    if (obsErrorSS.getCount() > Math.min(approxRuns/16, 155)) {
      
      AssertJUnit.assertArrayEquals(
          obsErrorSS.getMean().getDimensionality() == 4 ?
              fourZeros : twoZeros,
        obsErrorSS.getMean().toArray(), 3d * Math.sqrt(
            trueVehicleState.getMovementFilter().getObsCovar().normFrobenius()));
      
      AssertJUnit.assertArrayEquals(
          stateErrorSS.getMean().getDimensionality() == 4 ?
              fourZeros : twoZeros,
        stateErrorSS.getMean().toArray(), 
        3d * Math.sqrt(
            trueVehicleState.getMovementFilter().getOnRoadStateTransCovar().normFrobenius()));
      
      final Vector stateVelError = stateErrorSS.getMean().getDimensionality() == 4 ?
          AbstractRoadTrackingFilter.getVg().times(stateErrorSS.getMean())
          : VectorFactory.getDefault().copyValues(
              AbstractRoadTrackingFilter.getVr().times(stateErrorSS.getMean()).getElement(0));
      final Matrix trueV = trueVehicleState.getBelief().isOnRoad() ?
          AbstractRoadTrackingFilter.getVr() : AbstractRoadTrackingFilter.getVg();
      final double trueVelCovTol = 
          Math.sqrt(trueV.times(
            trueVehicleState.getMovementFilter().getOnRoadStateTransCovar()).times(
                trueV.transpose()).normFrobenius());
      
      AssertJUnit.assertArrayEquals(VectorFactory.getDefault().createVector(
          stateVelError.getDimensionality()).toArray(),
        stateVelError.toArray(),   2d * trueVelCovTol);
      
      AssertJUnit.assertArrayEquals(hasPriorOnVariances ? oneZero : fourZeros,
        onRoadCovErrorSS.getMean().toArray(), 
        0.7d *
            trueVehicleState.getMovementFilter().getQr().normFrobenius());
      
      AssertJUnit.assertArrayEquals(hasPriorOnVariances ? fourZeros : 
        sixteenZeros,
        offRoadCovErrorSS.getMean().toArray(), 
        0.7d *
            trueVehicleState.getMovementFilter().getQg().normFrobenius());
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
