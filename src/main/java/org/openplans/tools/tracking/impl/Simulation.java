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
import org.openplans.tools.tracking.impl.statistics.OnOffEdgeTransDirMulti;
import org.openplans.tools.tracking.impl.statistics.filters.AbstractRoadTrackingFilter;
import org.openplans.tools.tracking.impl.statistics.filters.StandardRoadTrackingFilter;
import org.openplans.tools.tracking.impl.statistics.filters.VehicleTrackingPathSamplerFilterUpdater;
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

    @Override
    public boolean equals(Object obj) {
      if (this == obj) {
        return true;
      }
      if (obj == null) {
        return false;
      }
      if (getClass() != obj.getClass()) {
        return false;
      }
      final SimulationParameters other = (SimulationParameters) obj;
      if (duration != other.duration) {
        return false;
      }
      if (endTime == null) {
        if (other.endTime != null) {
          return false;
        }
      } else if (!endTime.equals(other.endTime)) {
        return false;
      }
      if (frequency != other.frequency) {
        return false;
      }
      if (performInference != other.performInference) {
        return false;
      }
      if (startCoordinate == null) {
        if (other.startCoordinate != null) {
          return false;
        }
      } else if (!startCoordinate.equals(other.startCoordinate)) {
        return false;
      }
      if (startTime == null) {
        if (other.startTime != null) {
          return false;
        }
      } else if (!startTime.equals(other.startTime)) {
        return false;
      }
      if (stateParams == null) {
        if (other.stateParams != null) {
          return false;
        }
      } else if (!stateParams.equals(other.stateParams)) {
        return false;
      }
      return true;
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

    @Override
    public int hashCode() {
      final int prime = 31;
      int result = 1;
      result = prime * result + (int) (duration ^ (duration >>> 32));
      result =
          prime * result
              + ((endTime == null) ? 0 : endTime.hashCode());
      result =
          prime * result + (int) (frequency ^ (frequency >>> 32));
      result = prime * result + (performInference ? 1231 : 1237);
      result =
          prime
              * result
              + ((startCoordinate == null) ? 0 : startCoordinate
                  .hashCode());
      result =
          prime * result
              + ((startTime == null) ? 0 : startTime.hashCode());
      result =
          prime * result
              + ((stateParams == null) ? 0 : stateParams.hashCode());
      return result;
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

  private final String filterTypeName;
  private final VehicleTrackingPathSamplerFilterUpdater updater;

  public Simulation(String simulationName, OtpGraph graph,
    SimulationParameters simParameters, VehicleStateInitialParameters infParams) {

    this.simParameters = simParameters;
    this.parameters = infParams;
    this.filterTypeName =
        simParameters.getStateParams().getFilterTypeName();

    this.inferredGraph = graph;
    this.simulationName = simulationName;

    this.rng = new Random();
    if (parameters.getSeed() != 0l) {
      this.seed = parameters.getSeed();
    } else {
      this.seed = rng.nextLong();
    }

    this.rng.setSeed(seed);
    Observation initialObs = null;
    try {
      final Coordinate startCoord;
      if (this.simParameters.getStartCoordinate() == null) {
        startCoord = 
           GeoUtils.reverseCoordinates(this.inferredGraph.getTurnGraph().getExtent().centre());
      } else {
        startCoord = this.simParameters.getStartCoordinate();
      }
      
      Observation.remove(simulationName);
      initialObs =
          Observation.createObservation(this.simulationName,
              this.simParameters.getStartTime(),
              startCoord, null, null,
              null);
    } catch (final TimeOrderException e) {
      e.printStackTrace();
    }

    if (initialObs != null) {
      this.updater =
          new VehicleTrackingPathSamplerFilterUpdater(initialObs,
              graph, parameters);
      this.updater.setRandom(rng);
    } else {
      this.updater = null;
    }
  }

  public VehicleState computeInitialState() {

    /*
     * If the updater is null, then creation of the initial obs failed,
     * or something equally bad.
     * 
     */
    if (this.updater == null)
      return null;

    try {

      final Observation initialObs =
          this.updater.getInitialObservation();
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

      final StandardRoadTrackingFilter trackingFilter =
          new StandardRoadTrackingFilter(parameters.getObsVariance(),
              parameters.getOffRoadStateVariance(),
              parameters.getOnRoadStateVariance(),
              parameters.getInitialObsFreq());

      final OnOffEdgeTransDirMulti edgeTransDist =
          new OnOffEdgeTransDirMulti(inferredGraph,
              parameters.getOnTransitionProbs(),
              parameters.getOffTransitionProbs());

      final VehicleState vehicleState =
          new VehicleState(this.inferredGraph, initialObs,
              currentInferredEdge, trackingFilter, edgeTransDist, rng);

      return vehicleState;

    } catch (final NumberFormatException e) {
      e.printStackTrace();
    }

    return null;
  }

  public String getFilterTypeName() {
    return filterTypeName;
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

  /**
   * Samples an observation for the given state vector (velLocBelief).
   * Note: only the mean of the state vector is used.
   * @param velLocBelief
   * @param obsCov
   * @param edge
   * @return
   */
  public Vector sampleObservation(MultivariateGaussian velLocBelief,
    Matrix obsCov, PathEdge edge) {

    final MultivariateGaussian gbelief = velLocBelief.clone();
    AbstractRoadTrackingFilter.convertToGroundBelief(gbelief, edge);
    final Vector gMean =
        AbstractRoadTrackingFilter.getOg().times(gbelief.getMean());
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
    final OnOffEdgeTransDirMulti currentEdgeTrans =
        vehicleState.getEdgeTransitionDist();
    final PathEdge currentPathEdge =
        PathEdge.getEdge(vehicleState.getInferredEdge());

    /*
     * Run through the edges, predict movement and reset the belief.
     */
    final MultivariateGaussian predictiveBelief = currentLocBelief.clone();
    final InferredPath newPath =
        this.updater.traverseEdge(
            vehicleState.getEdgeTransitionDist(), predictiveBelief,
            currentPathEdge, vehicleState.getMovementFilter());
    

    final PathEdge newPathEdge =
        Iterables.getLast(newPath.getEdges());

    /*
     * Sample from the observation noise, and reset our state
     * position (the covariance really, since we're using the predicted
     * location, but want to keep the same covariance).
     */
    final MultivariateGaussian newBelief = predictiveBelief.clone();
    if (newPathEdge.isEmptyEdge()) {
      newBelief.setCovariance(vehicleState.getMovementFilter().getGroundFilter().getModelCovariance());
    } else {
      newBelief.setCovariance(vehicleState.getMovementFilter().getRoadFilter().getModelCovariance());
    }
    
    final Matrix gCov =
        vehicleState.getMovementFilter().getGroundFilter()
            .getMeasurementCovariance();
    final Vector thisLoc =
        sampleObservation(newBelief, gCov, newPathEdge);
    final Coordinate obsCoord =
        GeoUtils.convertToLatLon(thisLoc, vehicleState
            .getObservation().getObsPoint());
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
            vehicleState.getMovementFilter(), newBelief,
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

}
