package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrixFactoryMTJ;
import gov.sandia.cognition.math.matrix.mtj.decomposition.CholeskyDecompositionMTJ;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.Date;
import java.util.List;
import java.util.Random;
import java.util.Set;

import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.InferredPathEntry;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.statistics.EdgeTransitionDistributions;
import org.openplans.tools.tracking.impl.statistics.StandardRoadTrackingFilter;
import org.openplans.tools.tracking.impl.util.GeoUtils;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.routing.edgetype.StreetEdge;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Sets;
import com.vividsolutions.jts.geom.Coordinate;

public class Simulation {

  public static class SimulationParameters {

    private final Coordinate startCoordinate;
    private final Date startTime;
    private final Date endTime;
    private final long duration;
    private final long frequency;
    private final boolean performInference;
    private final VehicleStateInitialParameters stateParams;

    public SimulationParameters(Coordinate startCoordinate,
      Date startTime, long duration, long frequency,
      boolean performInference,
      VehicleStateInitialParameters stateParams) {
      this.stateParams = stateParams;
      this.performInference = performInference;
      this.frequency = frequency;
      this.startCoordinate = startCoordinate;
      this.startTime = startTime;
      this.endTime = new Date(startTime.getTime() + duration * 1000);
      this.duration = duration;
    }

    public long getDuration() {
      return duration;
    }

    public Date getEndTime() {
      return endTime;
    }

    public long getFrequency() {
      return frequency;
    }

    public Coordinate getStartCoordinate() {
      return startCoordinate;
    }

    public Date getStartTime() {
      return startTime;
    }

    public VehicleStateInitialParameters getStateParams() {
      return stateParams;
    }

    public boolean isPerformInference() {
      return performInference;
    }
  }

  private static final Logger _log = LoggerFactory
      .getLogger(Simulation.class);

  private final long seed;
  private final Random rng;
  private final OtpGraph inferredGraph;
  private final String simulationName;
  private final VehicleStateInitialParameters parameters;

  private int recordsProcessed = 0;

  private final SimulationParameters simParameters;
  private long localSeed;

  public Simulation(String simulationName, OtpGraph graph,
    SimulationParameters simParameters) {

    this.simParameters = simParameters;
    this.parameters = simParameters.getStateParams();

    this.inferredGraph = graph;
    this.simulationName = simulationName;

    this.rng = new Random();
    if (parameters.getSeed() != 0l) {
      this.seed = parameters.getSeed();
    } else {
      this.seed = rng.nextLong();
    }

    this.rng.setSeed(seed);
  }

  public VehicleState computeInitialState() {

    Observation initialObs;
    try {
      try {
        Observation.remove(simulationName);
        initialObs =
            Observation.createObservation(this.simulationName,
                this.simParameters.getStartTime(),
                this.simParameters.getStartCoordinate(), null, null,
                null);
      } catch (final TimeOrderException e) {
        e.printStackTrace();
        return null;
      }

      final List<InferredEdge> edges =
          Lists.newArrayList(InferredEdge.getEmptyEdge());
      for (final StreetEdge edge : this.inferredGraph.getNearbyEdges(
          initialObs.getProjectedPoint(), 25d)) {
        edges.add(this.inferredGraph.getInferredEdge(edge));
      }
      final Set<InferredPathEntry> evaluatedPaths = Sets.newHashSet();
      final InferredEdge currentInferredEdge =
          edges.get(rng.nextInt(edges.size()));
      for (final InferredEdge edge : edges) {
        final InferredPath thisPath;
        if (edge.isEmptyEdge()) {
          thisPath = InferredPath.getEmptyPath();
        } else {
          thisPath = InferredPath.getInferredPath(edge);
        }
        if (edge == currentInferredEdge) {
        }
        evaluatedPaths.add(new InferredPathEntry(thisPath, null,
            null, null, Double.NEGATIVE_INFINITY));
      }

      final VehicleState vehicleState =
          new VehicleState(this.inferredGraph, initialObs,
              currentInferredEdge, parameters, rng);

      return vehicleState;

    } catch (final NumberFormatException e) {
      e.printStackTrace();
    }

    return null;
  }

  public OtpGraph getInferredGraph() {
    return inferredGraph;
  }

  public VehicleStateInitialParameters getParameters() {
    return parameters;
  }

  public Random getRng() {
    return rng;
  }

  public long getSeed() {
    return seed;
  }

  public SimulationParameters getSimParameters() {
    return simParameters;
  }

  public String getSimulationName() {
    return simulationName;
  }

  public Vector sampleObservation(MultivariateGaussian velLocBelief,
    Matrix obsCov, PathEdge edge) {

    final MultivariateGaussian gbelief = velLocBelief.clone();
    StandardRoadTrackingFilter.convertToGroundBelief(gbelief, edge);
    final Vector gMean =
        StandardRoadTrackingFilter.getOg().times(gbelief.getMean());
    final Matrix covSqrt =
        CholeskyDecompositionMTJ.create(
            DenseMatrixFactoryMTJ.INSTANCE.copyMatrix(obsCov)).getR();
    final Vector thisStateSample =
        MultivariateGaussian.sample(gMean, covSqrt, rng);
    return thisStateSample;
  }

  private VehicleState sampleState(VehicleState vehicleState,
    long time) {

    vehicleState.getMovementFilter().setCurrentTimeDiff(
        this.simParameters.getFrequency());
    final MultivariateGaussian currentLocBelief =
        vehicleState.getBelief();
    final EdgeTransitionDistributions currentEdgeTrans =
        vehicleState.getEdgeTransitionDist();
    final PathEdge currentPathEdge =
        PathEdge.getEdge(vehicleState.getInferredEdge());

    /*
     * Run through the edges, predict movement and reset the belief.
     */
    this.localSeed = rng.nextLong();
    final InferredPath newPath =
        traverseEdge(vehicleState.getEdgeTransitionDist(),
            currentLocBelief, currentPathEdge,
            vehicleState.getMovementFilter());

    final PathEdge newPathEdge =
        Iterables.getLast(newPath.getEdges());

    /*
     * Sample from the state and observation noise
     */
    final Matrix gCov =
        vehicleState.getMovementFilter().getGroundFilter()
            .getMeasurementCovariance();
    final Vector thisLoc =
        sampleObservation(currentLocBelief, gCov, newPathEdge);
    final Coordinate obsCoord = GeoUtils.convertToLatLon(thisLoc);
    Observation thisObs;
    try {
      thisObs =
          Observation.createObservation(simulationName,
              new Date(time), obsCoord, null, null, null);
    } catch (final TimeOrderException e) {
      e.printStackTrace();
      return null;
    }

    final VehicleState newState =
        new VehicleState(this.inferredGraph, thisObs,
            vehicleState.getMovementFilter(), currentLocBelief,
            currentEdgeTrans, newPath, vehicleState);

    return newState;
  }

  public VehicleState stepSimulation(VehicleState currentState) {
    final long time =
        currentState.getObservation().getTimestamp().getTime()
            + this.simParameters.getFrequency() * 1000;
    final VehicleState vehicleState = sampleState(currentState, time);
    _log.info("processed simulation observation : "
        + recordsProcessed + ", " + time);
    recordsProcessed++;
    return vehicleState;
  }

  /**
   * This method samples a path, updates the belief to reflect that, and returns
   * the path.
   * 
   * @param edgeTransDist
   * @param belief
   * @param startEdge
   * @param movementFilter
   * @return
   */
  private InferredPath traverseEdge(
    EdgeTransitionDistributions edgeTransDist,
    final MultivariateGaussian belief, PathEdge startEdge,
    StandardRoadTrackingFilter movementFilter) {

    /*
     * We project the road path
     */
    rng.setSeed(this.localSeed);
    PathEdge currentEdge = startEdge;
    PathEdge previousEdge = null;
    final MultivariateGaussian newBelief = belief.clone();

    final List<PathEdge> currentPath = Lists.newArrayList();

    double distTraveled = 0d;
    Double totalDistToTravel = null;
    while (totalDistToTravel == null ||
    // the following case is when we're truly on the edge
        Math.abs(totalDistToTravel) > Math.abs(distTraveled)) {

      final List<InferredEdge> transferEdges = Lists.newArrayList();
      if (currentEdge.getInferredEdge() == InferredEdge
          .getEmptyEdge()) {
        final Vector projLocation =
            StandardRoadTrackingFilter.getOg().times(
                newBelief.getMean());
        for (final StreetEdge edge : this.inferredGraph
            .getNearbyEdges(projLocation,
                movementFilter.getObservationErrorAbsRadius())) {
          transferEdges.add(this.inferredGraph.getInferredEdge(edge));
        }
      } else {
        if (totalDistToTravel == null) {
          transferEdges.add(startEdge.getInferredEdge());
        } else {
          if (newBelief.getMean().getElement(0) < 0d) {
            transferEdges.addAll(currentEdge.getInferredEdge()
                .getIncomingTransferableEdges());
          } else if (newBelief.getMean().getElement(0) > 0d) {
            transferEdges.addAll(currentEdge.getInferredEdge()
                .getOutgoingTransferableEdges());
          } else {
            transferEdges.addAll(currentEdge.getInferredEdge()
                .getIncomingTransferableEdges());
            transferEdges.addAll(currentEdge.getInferredEdge()
                .getOutgoingTransferableEdges());
          }
          // Make sure we don't move back and forth
          transferEdges.remove(currentEdge.getInferredEdge());
        }
      }

      final InferredEdge sampledEdge =
          edgeTransDist.sample(rng, transferEdges,
              currentEdge == null ? startEdge.getInferredEdge()
                  : currentEdge.getInferredEdge());

      if (sampledEdge == InferredEdge.getEmptyEdge()) {

        if (totalDistToTravel == null) {
          /*
           * Off-road, so just return/add the empty path and be done
           */
          movementFilter.predict(newBelief,
              PathEdge.getEmptyPathEdge(), startEdge);
        } else {
          /*
           * This belief should/could extend past the length of the current
           * edge, so that the converted ground coordinates emulate driving
           * off of a road (most of the time, perhaps).
           */
          StandardRoadTrackingFilter.convertToGroundBelief(newBelief,
              currentEdge, true);
        }

        currentEdge = PathEdge.getEmptyPathEdge();
        currentPath.add(PathEdge.getEmptyPathEdge());
        break;
      }

      double direction =
          newBelief.getMean().getElement(0) >= 0d ? 1d : -1d;
      final PathEdge sampledPathEdge =
          PathEdge.getEdge(sampledEdge, previousEdge == null
              || previousEdge.isEmptyEdge() ? 0d : direction
              * previousEdge.getInferredEdge().getLength()
              + previousEdge.getDistToStartOfEdge(), direction < 0d);

      if (sampledPathEdge == null) {
        /*-
         * We have nowhere else to go, but we're not moving off of an edge, so 
         * we call this a stop.
         */
        newBelief.getMean().setElement(0,
            direction * currentEdge.getInferredEdge().getLength());
        newBelief.getMean().setElement(1, 0d);
        break;
      }

      if (totalDistToTravel == null) {
        /*
         * Predict the movement, i.e. distance and direction to travel. The mean
         * of this belief should be set to the true value, so the prediction is
         * exact.
         */

        /*
         * Since we might be just transferring onto an edge, check
         * first.
         */
        final PathEdge initialEdge =
            startEdge.isEmptyEdge() ? sampledPathEdge : startEdge;
        currentEdge = initialEdge;

        if (newBelief.getInputDimensionality() == 4) {
          StandardRoadTrackingFilter.convertToRoadBelief(newBelief,
              InferredPath.getInferredPath(initialEdge));
        }

        double previousLocation = newBelief.getMean().getElement(0);
        movementFilter.predict(newBelief, initialEdge, initialEdge);

        final Vector transStateSample =
            StandardRoadTrackingFilter.sampleMovementBelief(rng,
                newBelief.getMean(), movementFilter);
        newBelief.setMean(transStateSample);
        totalDistToTravel =
            newBelief.getMean().getElement(0) - previousLocation;

        double newLocation = newBelief.getMean().getElement(0);
        final double L = initialEdge.getInferredEdge().getLength();

        /*
         * Adjust reference locations to be the same, wrt the new
         * location's direction.
         */
        if (newLocation < 0d && previousLocation > 0d) {
          previousLocation = -L + previousLocation;
          newLocation = previousLocation + totalDistToTravel;
          newBelief.getMean().setElement(0, newLocation);
        } else if (newLocation >= 0d && previousLocation < 0d) {
          previousLocation = L + previousLocation;
          newLocation = previousLocation + totalDistToTravel;
          newBelief.getMean().setElement(0, newLocation);
        }

        /*
         * Get the distance we've covered to move off of this edge, if we
         * have moved off.
         */
        direction = totalDistToTravel >= 0d ? 1d : -1d;
        if (L < Math.abs(newLocation)) {
          final double r = L - Math.abs(previousLocation);
          distTraveled += r * direction;
        } else {
          distTraveled += totalDistToTravel;
        }
      } else {
        /*
         * Continue along edges
         */
        distTraveled +=
            direction * sampledPathEdge.getInferredEdge().getLength();
        currentEdge = sampledPathEdge;
      }
      previousEdge = currentEdge;
      currentPath.add(currentEdge);
    }

    //    if(!Iterables.getLast(currentPath).isEmptyEdge() && 
    //          !Iterables.getLast(currentPath).isOnEdge(newBelief.getMean().getElement(0))) {
    //      Iterables.getLast(currentPath).isOnEdge(newBelief.getMean().getElement(0));
    //    }

    assert (Iterables.getLast(currentPath).isEmptyEdge() || Iterables
        .getLast(currentPath).isOnEdge(
            newBelief.getMean().getElement(0)));

    belief.setMean(newBelief.getMean());
    belief.setCovariance(newBelief.getCovariance());
    return InferredPath.getInferredPath(currentPath,
        (totalDistToTravel != null && totalDistToTravel < 0d) ? true
            : false);
  }

}
