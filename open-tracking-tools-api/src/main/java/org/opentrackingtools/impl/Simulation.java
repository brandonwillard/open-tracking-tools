package org.opentrackingtools.impl;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.Date;
import java.util.Random;

import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.paths.states.PathState;
import org.opentrackingtools.graph.paths.states.impl.SimplePathStateBelief;
import org.opentrackingtools.statistics.distributions.impl.OnOffEdgeTransDirMulti;
import org.opentrackingtools.statistics.filters.vehicles.impl.VehicleTrackingPathSamplerFilterUpdater;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.statistics.impl.StatisticsUtil;
import org.opentrackingtools.util.GeoUtils;
import org.opentrackingtools.util.geom.ProjectedCoordinate;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

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
      this.endTime =
          new Date(startTime.getTime() + duration * 1000);
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
      final SimulationParameters other =
          (SimulationParameters) obj;
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
      } else if (!startCoordinate
          .equals(other.startCoordinate)) {
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
      result =
          prime * result
              + (int) (duration ^ (duration >>> 32));
      result =
          prime
              * result
              + ((endTime == null) ? 0 : endTime.hashCode());
      result =
          prime * result
              + (int) (frequency ^ (frequency >>> 32));
      result =
          prime * result + (performInference ? 1231 : 1237);
      result =
          prime
              * result
              + ((startCoordinate == null) ? 0
                  : startCoordinate.hashCode());
      result =
          prime
              * result
              + ((startTime == null) ? 0 : startTime
                  .hashCode());
      result =
          prime
              * result
              + ((stateParams == null) ? 0 : stateParams
                  .hashCode());
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
  private final InferenceGraph inferredGraph;
  private final String simulationName;
  private final VehicleStateInitialParameters infParameters;

  private int recordsProcessed = 0;

  private final SimulationParameters simParameters;

  private final String filterTypeName;
  private final VehicleTrackingPathSamplerFilterUpdater updater;

  public Simulation(String simulationName, InferenceGraph graph,
    SimulationParameters simParameters,
    VehicleStateInitialParameters infParams) {

    this.simParameters = simParameters;
    this.infParameters = infParams;
    this.filterTypeName =
        simParameters.getStateParams().getFilterTypeName();

    this.inferredGraph = graph;
    this.simulationName = simulationName;

    this.rng = new Random();
    if (this.simParameters.getStateParams().getSeed() != 0l) {
      this.seed =
          this.simParameters.getStateParams().getSeed();
    } else {
      this.seed = rng.nextLong();
    }

    this.rng.setSeed(seed);
    
    final Coordinate startCoord;
    if (this.simParameters.getStartCoordinate() == null) {
      startCoord =
          GeoUtils.reverseCoordinates(this.inferredGraph.
              getGPSGraphExtent().centre());
    } else {
      startCoord =
          this.simParameters.getStartCoordinate();
    }

    final ProjectedCoordinate obsPoint =
        GeoUtils.convertToEuclidean(startCoord);
    
    GpsObservation initialObs = new SimpleObservation(
              this.simulationName,
              this.simParameters.getStartTime(),
              startCoord, null, null, null,
              0, null, obsPoint);

    this.updater =
        new VehicleTrackingPathSamplerFilterUpdater(
            initialObs, graph,
            this.simParameters.getStateParams());
    this.updater.setRandom(rng);
  }

  public VehicleState computeInitialState() {

    /*
     * If the updater is null, then creation of the initial obs failed,
     * or something equally bad.
     * 
     * Otherwise, we just get the initial state from the updater.  
     * This method is effectively pointless unless we're doing something
     * special for simulations...
     * 
     */
    if (this.updater == null)
      return null;

    final VehicleState vehicleState =
        this.updater.createInitialParticles(1)
            .getMaxValueKey();

    return vehicleState;
  }

  public String getFilterTypeName() {
    return filterTypeName;
  }

  public InferenceGraph getInferredGraph() {
    return inferredGraph;
  }

  public VehicleStateInitialParameters getInfParameters() {
    return infParameters;
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
   * Samples an observation for the given state vector (velLocBelief). Note:
   * only the mean of the state vector is used.
   * 
   * @param velLocBelief
   * @param obsCov
   * @param edge
   * @return
   */
  public Vector sampleObservation(PathState newPathState,
    Matrix cov) {

    final Vector groundState = newPathState.getGroundState();
    final Vector gMean =
        AbstractRoadTrackingFilter.getOg().times(
            groundState);

    final Matrix covSqrt =
        StatisticsUtil.rootOfSemiDefinite(cov);

    final Vector thisStateSample =
        MultivariateGaussian.sample(gMean, covSqrt, rng);
    return thisStateSample;
  }

  private VehicleState sampleState(
    VehicleState vehicleState, long time) {

    vehicleState.getMovementFilter().setCurrentTimeDiff(
        this.simParameters.getFrequency());

    final OnOffEdgeTransDirMulti currentEdgeTrans =
        vehicleState.getEdgeTransitionDist().clone();

    /*
     * Run through the edges, predict movement and reset the belief.
     */
    final PathState newPathState =
        this.updater.sampleNextState(
            vehicleState.getEdgeTransitionDist(),
            vehicleState.getBelief(),
            vehicleState.getMovementFilter());

    final Matrix gCov =
        vehicleState.getMovementFilter().getObsCovar();
    final Vector thisLoc =
        sampleObservation(newPathState, gCov);

    final Coordinate obsCoord =
        GeoUtils.convertToLatLon(thisLoc, vehicleState
            .getObservation().getObsProjected());

    final int thisRecNum = vehicleState.getObservation().getRecordNumber();
    GpsObservation thisObs =
          new SimpleObservation(simulationName,
              new Date(time), obsCoord, null, null, null,
              thisRecNum, vehicleState.getObservation(), 
              new ProjectedCoordinate(GeoUtils.getTransform(obsCoord), 
                  GeoUtils.makeCoordinate(thisLoc), obsCoord));

    final SimplePathStateBelief newStateBelief =
        SimplePathStateBelief.getPathStateBelief(
            newPathState.getPath(),
            new MultivariateGaussian(newPathState
                .getGlobalState(), MatrixFactory
                .getDiagonalDefault().createMatrix(
                    newPathState.getGlobalState()
                        .getDimensionality(),
                    newPathState.getGlobalState()
                        .getDimensionality())));

    final VehicleState newState =
        new VehicleState(this.inferredGraph, thisObs,
            vehicleState.getMovementFilter(), newStateBelief,
            currentEdgeTrans, vehicleState);

    return newState;
  }

  public VehicleState stepSimulation(
    VehicleState currentState) {
    final long time =
        currentState.getObservation().getTimestamp()
            .getTime()
            + this.simParameters.getFrequency() * 1000;

    /*
     * TODO: set seed for debugging
     */
    this.updater.seed = this.rng.nextLong();

    final VehicleState vehicleState =
        sampleState(currentState, time);

    _log.info("processed simulation observation : "
        + recordsProcessed + ", " + time);

    recordsProcessed++;
    return vehicleState;
  }

}
