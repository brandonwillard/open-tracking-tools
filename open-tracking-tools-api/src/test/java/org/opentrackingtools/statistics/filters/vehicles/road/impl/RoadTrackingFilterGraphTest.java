package org.opentrackingtools.statistics.filters.vehicles.road.impl;

import org.testng.annotations.BeforeTest;
import org.testng.annotations.DataProvider;
import org.testng.annotations.Test;
import org.testng.AssertJUnit;
import org.testng.internal.junit.ArrayAsserts;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.geotools.factory.FactoryRegistryException;
import org.geotools.referencing.operation.projection.ProjectionException;

import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.Date;
import java.util.List;
import java.util.Random;

import gov.sandia.cognition.math.MutableDouble;
import gov.sandia.cognition.math.RingAccumulator;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorEntry;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian.SufficientStatistic;

import org.opengis.referencing.FactoryException;
import org.opengis.referencing.NoSuchAuthorityCodeException;
import org.opengis.referencing.operation.NoninvertibleTransformException;
import org.opengis.referencing.operation.TransformException;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.impl.GenericJTSGraph;
import org.opentrackingtools.graph.paths.impl.TrackingTestUtils;
import org.opentrackingtools.impl.Simulation;
import org.opentrackingtools.impl.Simulation.SimulationParameters;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.distributions.impl.OnOffEdgeTransDistribution;
import org.opentrackingtools.statistics.filters.vehicles.VehicleTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.impl.VehicleTrackingPLFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.util.TrueObservation;
import org.opentrackingtools.GpsObservation;

import au.com.bytecode.opencsv.CSVWriter;

import com.beust.jcommander.internal.Lists;
import com.google.common.base.Stopwatch;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineString;

public class RoadTrackingFilterGraphTest {
  
  final Logger log = Logger.getLogger(RoadTrackingFilterGraphTest.class);
  
  private InferenceGraph graph;
  private Coordinate startCoord;
  private Matrix avgTransform;
  private CSVWriter writer;
  private Simulation sim;
  private static final double[] sixteenZeros = VectorFactory.getDefault()
      .createVector(16).toArray() ;
  private static final double[] fourZeros = new double[] {0, 0, 0, 0};
  private static final double[] twoZeros = new double[] {0, 0};
  private static final double[] oneZero = new double[] {0};
  
  @BeforeTest
  public void setUp() throws NoSuchAuthorityCodeException, FactoryRegistryException, FactoryException, IOException {
    
    log.setLevel(Level.DEBUG);
    
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
                6.25e-4), 20, VectorFactory.getDefault()
                .createVector2D(6.25e-4, 6.25e-4), 20,
            VectorFactory.getDefault().createVector2D(1d,
                Double.MAX_VALUE), VectorFactory.getDefault()
                .createVector2D(Double.MAX_VALUE, 1d),
            VehicleTrackingPLFilter.class.getName(), 
            ForwardMovingRoadTrackingFilter.class.getName(), 
            25, 30, 2159585l),
        new VehicleStateInitialParameters(VectorFactory
            .getDefault().createVector2D(70d, 70d), 20,
            VectorFactory.getDefault().createVector1D(
                6.25e-4), 20, VectorFactory.getDefault()
                .createVector2D(6.25e-4, 6.25e-4), 20,
            VectorFactory.getDefault().createVector2D(1d,
                Double.MAX_VALUE), VectorFactory.getDefault()
                .createVector2D(Double.MAX_VALUE, 1d),
            VehicleTrackingPLFilter.class.getName(), 
            ErrorEstimatingRoadTrackingFilter.class.getName(), 
            25, 30, 2159585l),
        Boolean.FALSE,
        36000
        }
//        ,{
//          /*
//           * Mixed 
//           */
//        new VehicleStateInitialParameters(VectorFactory
//            .getDefault().createVector2D(70d, 70d), 20,
//            VectorFactory.getDefault().createVector1D(
//                6.25e-4), 20, VectorFactory.getDefault()
//                .createVector2D(6.25e-4, 6.25e-4), 20,
//            VectorFactory.getDefault().createVector2D(0.5d,
//                0.5d), VectorFactory.getDefault()
//                .createVector2D(0.5d, 0.5d),
//            VehicleTrackingPLFilter.class.getName(), 
//            ErrorEstimatingRoadTrackingFilter.class.getName(), 
//            25, 30, 2159585l),
//        new VehicleStateInitialParameters(VectorFactory
//            .getDefault().createVector2D(70d, 70d), 20,
//            VectorFactory.getDefault().createVector1D(
//                6.25e-4), 20, VectorFactory.getDefault()
//                .createVector2D(6.25e-4, 6.25e-4), 20,
//            VectorFactory.getDefault().createVector2D(70d,
//                30d), VectorFactory.getDefault()
//                .createVector2D(30d, 70d),
//            VehicleTrackingPLFilter.class.getName(), 
//            ErrorEstimatingRoadTrackingFilter.class.getName(), 
//            25, 30, 2159585l),
//        Boolean.FALSE,
//        36000
//        }
    };
  }
  
  @Test(dataProvider="initialStateData")
  public void runSimulation(VehicleStateInitialParameters simInitialParams,
    VehicleStateInitialParameters filterInitialParams,
    boolean generalizeMoveDiff, long duration) throws NoninvertibleTransformException, TransformException, ClassNotFoundException, SecurityException, NoSuchMethodException, IllegalArgumentException, InstantiationException, IllegalAccessException, InvocationTargetException, IOException {
    
    writer = new CSVWriter(new FileWriter("road-tracking-test-results.csv"));
    SimulationParameters simParams = new SimulationParameters(
        startCoord, new Date(0l), duration, 
        simInitialParams.getInitialObsFreq(), false, false, 
        simInitialParams);
    
    sim = new Simulation("test-sim", graph, simParams, 
        simInitialParams);
    
    long time = sim.getSimParameters().getStartTime().getTime();
    
    VehicleState trueVehicleState = sim.computeInitialState();
    
    Class<?> filterType = Class.forName(filterInitialParams.getParticleFilterTypeName());
      
    Constructor<?> ctor = filterType.getConstructor(GpsObservation.class, 
          InferenceGraph.class,
          VehicleStateInitialParameters.class, 
          Boolean.class, Random.class);
      
    Random rng;
    if (filterInitialParams.getSeed() != 0)
      rng = new Random(filterInitialParams.getSeed());
    else
      rng = new Random();
    
    @SuppressWarnings("unchecked")
    VehicleTrackingFilter<GpsObservation, VehicleState> filter = 
        (VehicleTrackingFilter) ctor.newInstance(
          new TrueObservation(trueVehicleState.getObservation(), 
              trueVehicleState), graph, filterInitialParams, true, rng);
    
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
    
    final RingAccumulator<MutableDouble> averager =
        new RingAccumulator<MutableDouble>();
    final Stopwatch watch = new Stopwatch();
    
    final long approxRuns = sim.getSimParameters().getDuration()/
        sim.getSimParameters().getFrequency();
    
    updateAndCheckStats(vehicleStateDist, trueVehicleState, obsErrorSS, stateErrorSS, 
        obsCovErrorSS, onRoadCovErrorSS, offRoadCovErrorSS, transitionsSS, 
        generalizeMoveDiff);
    
    do {
      try {
        trueVehicleState = sim.stepSimulation(trueVehicleState);
        
        if (trueVehicleState.getMovementFilter() instanceof ForwardMovingRoadTrackingFilter) {
          if (trueVehicleState.getBelief().isOnRoad()) {
            for (VectorEntry entry : trueVehicleState.getBelief().getGlobalState()) {
              AssertJUnit.assertTrue(entry.getValue() >= 0d);
            }
          }
        }
    
        watch.reset();
        watch.start();
        
        filter.update(vehicleStateDist, 
            new TrueObservation(trueVehicleState.getObservation(), 
                trueVehicleState));
        
        watch.stop();
        averager.accumulate(new MutableDouble(watch.elapsedMillis()));
        
        if (averager.getCount() > 1 && averager.getCount() % 20 == 0)
          log.info("avg. records per sec = " + 1000d
              / averager.getMean().value);

        
        updateAndCheckStats(vehicleStateDist, trueVehicleState, obsErrorSS, stateErrorSS, 
            obsCovErrorSS, onRoadCovErrorSS, offRoadCovErrorSS, transitionsSS, 
            generalizeMoveDiff);
        
        time = trueVehicleState.getObservation().getTimestamp().getTime();
      } catch (ProjectionException ex) {
        trueVehicleState = resetState(trueVehicleState);
//        if (vehicleState.getParentState() != null)
//          vehicleState.setParentState(resetState(vehicleState.getParentState()));
        log.warn("Outside of projection!  Flipped state velocities");
      }
      
    } while (time < sim.getSimParameters().getEndTime().getTime());
    
    AssertJUnit.assertTrue(transitionsSS.getCount() > 0.95d * approxRuns );
  }


  private void updateAndCheckStats(DataDistribution<VehicleState> vehicleStateDist, 
    VehicleState trueVehicleState,
    SufficientStatistic obsErrorSS, 
    SufficientStatistic stateErrorSS, 
    SufficientStatistic obsCovErrorSS,
    SufficientStatistic onRoadCovErrorSS,
    SufficientStatistic offRoadCovErrorSS,
    SufficientStatistic transitionsSS, boolean generalizeMoveDiff) throws IOException {
    
    SufficientStatistic stateMeanStat = 
        new MultivariateGaussian.SufficientStatistic();
    SufficientStatistic obsCovStat = 
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
      
      if (state.getBelief().isOnRoad() && trueVehicleState.getBelief().isOnRoad()
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
        obsCovStat.update(obsCovMean.convertToVector());
        final Matrix onRoadCovMean = ((ErrorEstimatingRoadTrackingFilter) state.getMovementFilter())
          .getOnRoadStateVariancePrior().getMean();
        onRoadCovStat.update(onRoadCovMean.convertToVector());
        final Matrix offRoadCovMean = ((ErrorEstimatingRoadTrackingFilter) state.getMovementFilter())
          .getOffRoadStateVariancePrior().getMean();
        offRoadCovStat.update(offRoadCovMean.convertToVector());
      } else {
        obsCovStat.update(state.getMovementFilter().getObsCovar()
            .convertToVector());
        onRoadCovStat.update(state.getMovementFilter().getOnRoadStateTransCovar()
            .convertToVector());
        offRoadCovStat.update(state.getMovementFilter().getOffRoadStateTransCovar()
            .convertToVector());
      }
      final VehicleState parentState = state.getParentState();
      if (parentState != null) {
        
        Vector transType = OnOffEdgeTransDistribution.getTransitionType(
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
      
    log.debug("truePathsFound=" + truePathsFound);
    log.debug("numOnTrueEdges=" + numOnTrueEdge);
      
    final Vector obsError = trueVehicleState.getObservation().getProjectedPoint().
        minus(AbstractRoadTrackingFilter.getOg().times(stateMeanStat.getMean()));
    final Vector stateError = trueVehicleState.getBelief().getGroundState().
        minus(stateMeanStat.getMean());
    
    final Vector obsCovError;   
    final Vector onRoadCovError;
    final Vector offRoadCovError;
    if (hasPriorOnVariances) {
      obsCovError = obsCovStat.getMean().minus(
          trueVehicleState.getMovementFilter().getObsCovar().convertToVector());
      onRoadCovError = onRoadCovStat.getMean().minus(
          trueVehicleState.getMovementFilter().getQr().convertToVector());
      offRoadCovError = offRoadCovStat.getMean().minus(
          trueVehicleState.getMovementFilter().getQg().convertToVector());
    } else {
      obsCovError = obsCovStat.getMean().minus(
          trueVehicleState.getMovementFilter().getObsCovar().convertToVector());
      onRoadCovError = onRoadCovStat.getMean().minus(
          trueVehicleState.getMovementFilter().getOnRoadStateTransCovar().convertToVector());
      offRoadCovError = offRoadCovStat.getMean().minus(
          trueVehicleState.getMovementFilter().getOffRoadStateTransCovar().convertToVector());
    }
    
    log.debug("obsError=" + obsError);
    log.debug("stateError=" + stateError);
    if (hasPriorOnVariances) {
      log.debug("obsCovMean=" + obsCovStat.getMean());
      log.debug("onRoadCovMean=" + onRoadCovStat.getMean());
  //    log.debug("offRoadCovMean=" + offRoadCovStat.getMean());
    }
    
    if (transitionStat.getMean() != null) {
      final Vector trueTransProbs = trueVehicleState.getEdgeTransitionDist()
          .getEdgeMotionTransPrior().getMean().stack(
              trueVehicleState.getEdgeTransitionDist().
              getFreeMotionTransPrior().getMean());
      
      transitionsSS.update(transitionStat.getMean().minus(trueTransProbs));
//     log.debug("transitionMean=" + transitionStat.getMean());
    }
    
    obsErrorSS.update(obsError);
    stateErrorSS.update(stateError);
    obsCovErrorSS.update(obsCovError);
    onRoadCovErrorSS.update(onRoadCovError);
    offRoadCovErrorSS.update(offRoadCovError);
      
    if (!hasPriorOnVariances) {
      log.debug("obsErrorSS=" + obsErrorSS.getMean());
      log.debug("stateErrorSS=" + stateErrorSS.getMean());
//      log.debug("obsCovErrorSS=" + obsCovErrorSS.getMean());
//      log.debug("onRoadCovErrorSS=" + onRoadCovErrorSS.getMean());
//      log.debug("offRoadCovErrorSS=" + offRoadCovErrorSS.getMean());
//      log.debug("transitionErrorSS=" + transitionsSS.getMean());
    }
    
    final long approxRuns = sim.getSimParameters().getDuration()/
        sim.getSimParameters().getFrequency();
    if (obsErrorSS.getCount() > Math.min(approxRuns/16, 25)) {
      
      List<String> csvResults = Lists.newArrayList();
      
      ArrayAsserts.assertArrayEquals(
          obsErrorSS.getMean().getDimensionality() == 4 ?
              fourZeros : twoZeros,
        obsErrorSS.getMean().toArray(), 3d * Math.sqrt(
            trueVehicleState.getMovementFilter().getObsCovar().normFrobenius()));
      
      ArrayAsserts.assertArrayEquals(
          stateErrorSS.getMean().getDimensionality() == 4 ?
              fourZeros : twoZeros,
        stateErrorSS.getMean().toArray(), 
        5d * Math.sqrt(trueVehicleState.getMovementFilter()
            .getOnRoadStateTransCovar().normFrobenius()));
      
      csvResults.add(stateMeanStat.getMean().toString());
      csvResults.add(stateMeanStat.getCovariance().convertToVector().toString());
      
      final Vector stateVelError = stateErrorSS.getMean().getDimensionality() == 4 ?
          AbstractRoadTrackingFilter.getVg().times(stateErrorSS.getMean())
          : VectorFactory.getDefault().copyValues(
              AbstractRoadTrackingFilter.getVr().times(stateErrorSS.getMean()).getElement(0));
      final Matrix trueV = trueVehicleState.getBelief().isOnRoad() ?
          AbstractRoadTrackingFilter.getVr() : AbstractRoadTrackingFilter.getVg();
      final Matrix trueTransCov = trueVehicleState.getBelief().isOnRoad() ?
          trueVehicleState.getMovementFilter().getOnRoadStateTransCovar() :
            trueVehicleState.getMovementFilter().getOffRoadStateTransCovar();
      final double trueVelCovTol = 
          Math.sqrt(trueV.times(trueTransCov).times(
                trueV.transpose()).normFrobenius());
      
      ArrayAsserts.assertArrayEquals(VectorFactory.getDefault().createVector(
          stateVelError.getDimensionality()).toArray(),
        stateVelError.toArray(),   4d * trueVelCovTol);
      
      ArrayAsserts.assertArrayEquals(hasPriorOnVariances ? oneZero : fourZeros,
        onRoadCovError.toArray(), 
        0.7d *
            trueVehicleState.getMovementFilter().getQr().normFrobenius());
      
      csvResults.add(onRoadCovStat.getMean().toString());
      csvResults.add(onRoadCovStat.getCovariance().convertToVector().toString());
      
      ArrayAsserts.assertArrayEquals(fourZeros,
        obsCovError.toArray(), 
        0.7d *
            trueVehicleState.getMovementFilter().getObsCovar().normFrobenius());
      
      csvResults.add(obsCovStat.getMean().toString());
      csvResults.add(obsCovStat.getCovariance().convertToVector().toString());
      
      ArrayAsserts.assertArrayEquals(hasPriorOnVariances ? fourZeros : 
        sixteenZeros,
        offRoadCovError.toArray(), 
        0.7d *
            trueVehicleState.getMovementFilter().getQg().normFrobenius());
      
      csvResults.add(offRoadCovStat.getMean().toString());
      csvResults.add(offRoadCovStat.getCovariance().convertToVector().toString());
      
      writer.writeNext(csvResults.toArray(new String[csvResults.size()]));
      writer.flush();
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
