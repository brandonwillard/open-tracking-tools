package inference;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrixFactoryMTJ;
import gov.sandia.cognition.math.matrix.mtj.decomposition.CholeskyDecompositionMTJ;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.distribution.DefaultDataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.text.ParseException;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Random;
import java.util.Set;

//import org.slf4j.Logger;
//import org.slf4j.LoggerFactory;

import org.opengis.referencing.operation.TransformException;
import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;
import org.openplans.tools.tracking.impl.EdgeTransitionDistributions;
import org.openplans.tools.tracking.impl.InferredGraph;
import org.openplans.tools.tracking.impl.InferredPath;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.PathEdge;
import org.openplans.tools.tracking.impl.SnappedEdges;
import org.openplans.tools.tracking.impl.StandardRoadTrackingFilter;
import org.openplans.tools.tracking.impl.TimeOrderException;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.util.GeoUtils;
import org.opentripplanner.routing.graph.Edge;

import play.*;


import akka.actor.UntypedActor;

import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableSet;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Sets;
import com.vividsolutions.jts.geom.Coordinate;

import controllers.Api;

public class Simulation {
  
//  private static final Logger _log = LoggerFactory.getLogger(Simulation.class);
  
  private final Coordinate startCoordinates;
  private final Date startTime;
  private final Date endTime;

  private final List<InferenceResultRecord> results = Lists.newArrayList();
  private final int duration;
  private final int frequency;
  
  private final Random rng = new Random();
  
  private final InferredGraph inferredGraph;

  private final String simulationName;
  
  public Simulation() {
    
    this.startCoordinates = new Coordinate(10.300252, 123.90609);
    this.startTime = new Date(1325570441000l);
    this.duration = 60*60;
    this.endTime = new Date(startTime.getTime() + duration*1000);
    this.frequency = 30;
    this.inferredGraph = new InferredGraph(Api.getGraph());
    this.simulationName = "sim-" + this.startTime.getTime(); 
  }

  /**
   * Use this sampling method to beat the inherent degeneracy of our 
   * state covariance.
   * @param vehicleState
   * @return
   */
  public Vector sampleBelief(VehicleState vehicleState) {
    final boolean isRoad = vehicleState.getBelief().getInputDimensionality() == 2;
    final Matrix Q = isRoad ? vehicleState.getMovementFilter().getQr()
        : vehicleState.getMovementFilter().getQg();
    
    Matrix covSqrt = CholeskyDecompositionMTJ.create(
        DenseMatrixFactoryMTJ.INSTANCE.copyMatrix( Q ) ).getR();
    Vector thisStateSample = MultivariateGaussian.sample(VectorFactory.getDefault().createVector(2), 
        covSqrt, rng);
    if (isRoad) {
      final double length = vehicleState.getInferredEdge().getLength();
      /*
       * Let's keep this sample on the road
       */
      if (thisStateSample.getElement(0) > length) {
        thisStateSample.setElement(0, length);
      } else if (thisStateSample.getElement(0) < 0) {
        thisStateSample.setElement(0, 0d);
      }
    }
    final Matrix Gamma = vehicleState.getMovementFilter().getCovarianceFactor(isRoad);
    return Gamma.times(thisStateSample).plus(vehicleState.getBelief().getMean());
  }
  
  public Vector sampleObservation(VehicleState vehicleState, PathEdge edge) {
    
    final MultivariateGaussian gbelief =  vehicleState.getBelief().clone();
    StandardRoadTrackingFilter.convertToGroundBelief(gbelief, edge);
    final Vector gMean = StandardRoadTrackingFilter.getOg().times(gbelief.getMean());
    final Matrix gCov = vehicleState.getMovementFilter().getGroundFilter().getMeasurementCovariance();
    Matrix covSqrt = CholeskyDecompositionMTJ.create(
        DenseMatrixFactoryMTJ.INSTANCE.copyMatrix( gCov ) ).getR();
    Vector thisStateSample = MultivariateGaussian.sample(gMean, covSqrt, rng);
    return thisStateSample;
  }
  
  public void runSimulation() {
    
    Observation initialObs;
    try {
      try {
        initialObs = Observation.createObservation(this.simulationName, startTime, 
            startCoordinates, null, null, null);
      } catch (TimeOrderException e) {
        e.printStackTrace();
        return;
      }
      
      final SnappedEdges initialEdges = Api.getGraph().snapToGraph(null, initialObs.getObsCoords());
     
      /*
       * Choose a random edge to start on
       */
      List<InferredEdge> edges = Lists.newArrayList(InferredGraph.getEmptyEdge());
      for (Edge edge : initialEdges.getSnappedEdges()) {
        edges.add(inferredGraph.getInferredEdge(edge));
      }
      final InferredEdge currentInferredEdge = edges.get(rng.nextInt(edges.size()));
      
          
      InferredPath currentPath = currentInferredEdge == InferredGraph.getEmptyEdge() ?
          InferredPath.getEmptyPath() : new InferredPath(currentInferredEdge);
          
      PathEdge currentPathEdge = Iterables.getOnlyElement(currentPath.getEdges());
      
      VehicleState vehicleState = new VehicleState(this.inferredGraph, initialObs, currentInferredEdge);
      Vector thisStateSample = sampleBelief(vehicleState);
      vehicleState.getBelief().setMean(thisStateSample);
      
      for (long time = startTime.getTime() + frequency*1000; 
          time < endTime.getTime(); time += frequency*1000) {
        vehicleState.getMovementFilter().setCurrentTimeDiff(frequency);
        
        final MultivariateGaussian currentLocBelief = vehicleState.getBelief();
        final EdgeTransitionDistributions currentEdgeTrans = vehicleState.getEdgeTransitionDist();
        
        /*
         * Run through the edges, predict movement and reset the belief.
         */
        currentPath = traverseEdge(vehicleState.getEdgeTransitionDist(), 
            currentLocBelief, currentPathEdge, vehicleState.getMovementFilter());
        
        /*
         * Now that we've propagated the belief, sample it again, so that we
         * include some transition noise.
         */
        Vector transStateSample = sampleBelief(vehicleState);
        vehicleState.getBelief().setMean(transStateSample);
        
        currentPathEdge = Iterables.getLast(currentPath.getEdges());
        
        /*
         * Sample from the state and observation noise
         */
        Vector thisLoc = sampleObservation(vehicleState, currentPathEdge);
        Coordinate obsCoord = GeoUtils.convertToLatLon(thisLoc);
        Observation thisObs;
        try {
          thisObs = Observation.createObservation(simulationName, new Date(time), 
            obsCoord, null, null, null);
        } catch (TimeOrderException e) {
          e.printStackTrace();
          continue;
        }
        InferenceResultRecord result = InferenceResultRecord.createInferenceResultRecord(thisObs, 
            currentLocBelief, currentPath.getEdges());
        InferenceService.addSimulationRecords(simulationName, result);
        
        Logger.info("processed simulation observation :" + thisObs);
        vehicleState = new VehicleState(this.inferredGraph, thisObs, 
            vehicleState.getMovementFilter(), 
            currentLocBelief, currentEdgeTrans,
            currentPathEdge, currentPath, vehicleState);
      }
      
    } catch (NumberFormatException e) {
      e.printStackTrace();
    }
    
  }

  /**
   * This method samples a path, updates the belief to reflect that, and returns the path.
   * @param edgeTransDist
   * @param belief
   * @param startEdge
   * @param movementFilter
   * @return
   */
  private InferredPath traverseEdge(EdgeTransitionDistributions edgeTransDist, 
    final MultivariateGaussian belief, PathEdge startEdge, 
    StandardRoadTrackingFilter movementFilter) {
    
    /*
     * We project the road path 
     */
    PathEdge currentEdge = startEdge;
      
    List<PathEdge> currentPath = Lists.newArrayList();
    
    double distTraveled = 0d;
    Double totalDistToTravel = null;
    while (totalDistToTravel == null || 
        // the following case is when we're truly on the edge
        totalDistToTravel >= currentEdge.getDistToStartOfEdge() + currentEdge.getInferredEdge().getLength()) {
      
      
      List<InferredEdge> transferEdges = Lists.newArrayList();
      if (currentEdge.getInferredEdge() == InferredGraph.getEmptyEdge()) {
        transferEdges.addAll(this.inferredGraph.getNearbyEdges(belief.getMean()));
      } else {
        if (belief.getMean().getElement(0) < 0d) {
          transferEdges.addAll(currentEdge.getInferredEdge().getIncomingTransferableEdges());
        } else if (belief.getMean().getElement(0) > 0d){
          transferEdges.addAll(currentEdge.getInferredEdge().getOutgoingTransferableEdges());
        } else {
          transferEdges.addAll(currentEdge.getInferredEdge().getIncomingTransferableEdges());
          transferEdges.addAll(currentEdge.getInferredEdge().getOutgoingTransferableEdges());
        }
        transferEdges.add(currentEdge.getInferredEdge());
      }
      
      InferredEdge sampledEdge = edgeTransDist.sample(rng, transferEdges, 
          currentEdge.getInferredEdge());
      
      if (sampledEdge == InferredGraph.getEmptyEdge()) {
        
        /*
         * Off-road, so just return/add the empty path and be done
         */
        movementFilter.predict(belief, PathEdge.getEmptyPathEdge(), 
            currentEdge);
        
        if (currentPath.isEmpty()) {
          return InferredPath.getEmptyPath();
        } else {
          currentPath.add(PathEdge.getEmptyPathEdge());
          return new InferredPath(ImmutableList.copyOf(currentPath), distTraveled);
        }
      }
      
      PathEdge sampledPathEdge = PathEdge.getEdge(sampledEdge, distTraveled);
      
      if (totalDistToTravel == null) {
        /*
         * Predict the movement, i.e. distance and direction to travel.
         * The mean of this belief should be set to the true value, so
         * the prediction is exact.
         */
        if (belief.getInputDimensionality() == 4) {
          StandardRoadTrackingFilter.invertProjection(belief, sampledPathEdge);
        }
        
        movementFilter.predict(belief, null, null);
        
        totalDistToTravel = belief.getMean().getElement(0);
      }
      
      final double direction = belief.getMean().getElement(0) >= 0d ? 1d : -1d;
      
      if (sampledPathEdge == null) {
        /*-
         * We have nowhere else to go, but we're not moving off of an edge, so 
         * we call this a stop.
         */
        belief.getMean().setElement(0, direction * currentEdge.getInferredEdge().getLength());
        belief.getMean().setElement(1, 0d);
        
        break;
        
      } 
      
      /*
       * Continue along edges
       */
      distTraveled += direction * currentEdge.getInferredEdge().getLength();
      currentEdge = sampledPathEdge;
      currentPath.add(sampledPathEdge);
    
    }
    
    /*
     * If we're here, then we've safely landed on an edge from our total distance projected.
     * Reset the belief.
     */
    belief.getMean().setElement(0, belief.getMean().getElement(0) - currentEdge.getDistToStartOfEdge());
    
    return new InferredPath(ImmutableList.copyOf(currentPath));
  }

  public String getSimulationName() {
    return simulationName;
  }

  public static class SimulationActor extends UntypedActor {
    @Override
    public void onReceive(Object arg0) throws Exception {
      // TODO this is a lame way to get concurrency.  fix this
      Simulation sim = (Simulation) arg0;
      sim.runSimulation();
    }
  }
  
}
