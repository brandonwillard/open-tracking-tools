package org.opentrackingtools.util;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.lang.reflect.InvocationTargetException;
import java.util.Date;
import java.util.Random;

import org.opengis.referencing.operation.NoninvertibleTransformException;
import org.opengis.referencing.operation.TransformException;
import org.opentrackingtools.VehicleStateInitialParameters;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.ProjectedCoordinate;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.updater.VehicleStateBootstrapUpdater;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.vividsolutions.jts.geom.Coordinate;

public class Simulation {

  public static class SimulationParameters {

    protected final boolean canMoveBackward;
    protected final long duration;
    protected final Date endTime;
    protected final double frequency;
    protected final boolean performInference;
    protected final Coordinate startCoordinate;
    protected final Date startTime;
    protected final VehicleStateInitialParameters stateParams;

    public SimulationParameters(Coordinate startCoordinate,
      Date startTime, long duration, double frequency,
      boolean performInference, boolean canMoveBackward,
      VehicleStateInitialParameters stateParams) {
      this.stateParams = stateParams;
      this.performInference = performInference;
      this.frequency = frequency;
      this.startCoordinate = startCoordinate;
      this.startTime = startTime;
      this.endTime = new Date(startTime.getTime() + duration * 1000);
      this.duration = duration;
      this.canMoveBackward = canMoveBackward;
    }

    @Override
    public boolean equals(Object obj) {
      if (this == obj)
        return true;
      if (obj == null)
        return false;
      if (getClass() != obj.getClass())
        return false;
      SimulationParameters other = (SimulationParameters) obj;
      if (canMoveBackward != other.canMoveBackward)
        return false;
      if (duration != other.duration)
        return false;
      if (endTime == null) {
        if (other.endTime != null)
          return false;
      } else if (!endTime.equals(other.endTime))
        return false;
      if (Double.doubleToLongBits(frequency) != Double
          .doubleToLongBits(other.frequency))
        return false;
      if (performInference != other.performInference)
        return false;
      if (startCoordinate == null) {
        if (other.startCoordinate != null)
          return false;
      } else if (!startCoordinate.equals(other.startCoordinate))
        return false;
      if (startTime == null) {
        if (other.startTime != null)
          return false;
      } else if (!startTime.equals(other.startTime))
        return false;
      if (stateParams == null) {
        if (other.stateParams != null)
          return false;
      } else if (!stateParams.equals(other.stateParams))
        return false;
      return true;
    }

    public long getDuration() {
      return this.duration;
    }

    public Date getEndTime() {
      return this.endTime;
    }

    public double getFrequency() {
      return this.frequency;
    }

    public Coordinate getStartCoordinate() {
      return this.startCoordinate;
    }

    public Date getStartTime() {
      return this.startTime;
    }

    public VehicleStateInitialParameters getStateParams() {
      return this.stateParams;
    }

    @Override
    public int hashCode() {
      final int prime = 31;
      int result = 1;
      result = prime * result + (canMoveBackward ? 1231 : 1237);
      result = prime * result + (int) (duration ^ (duration >>> 32));
      result =
          prime * result
              + ((endTime == null) ? 0 : endTime.hashCode());
      long temp;
      temp = Double.doubleToLongBits(frequency);
      result = prime * result + (int) (temp ^ (temp >>> 32));
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

    public boolean isCanMoveBackward() {
      return this.canMoveBackward;
    }

    public boolean isPerformInference() {
      return this.performInference;
    }
  }

  private static final Logger _log = LoggerFactory
      .getLogger(Simulation.class);

  private final InferenceGraph inferredGraph;
  private final VehicleStateInitialParameters infParameters;
  private int recordsProcessed = 0;
  private final Random rng;
  private final long seed;

  private final SimulationParameters simParameters;

  private final String simulationName;

  public int getRecordsProcessed() {
    return recordsProcessed;
  }

  public VehicleStateBootstrapUpdater<GpsObservation> getUpdater() {
    return updater;
  }

  private final VehicleStateBootstrapUpdater<GpsObservation> updater;

  public Simulation(String simulationName, InferenceGraph graph,
    SimulationParameters simParameters,
    VehicleStateInitialParameters infParams) {

    this.simParameters = simParameters;
    this.infParameters = infParams;

    this.inferredGraph = graph;
    this.simulationName = simulationName;

    this.rng = new Random();
    if (this.simParameters.getStateParams().getSeed() != 0l) {
      this.seed = this.simParameters.getStateParams().getSeed();
    } else {
      this.seed = this.rng.nextLong();
    }

    this.rng.setSeed(this.seed);

    final Coordinate startCoord;
    if (this.simParameters.getStartCoordinate() == null) {
      startCoord = this.inferredGraph.getGPSGraphExtent().centre();
    } else {
      startCoord = this.simParameters.getStartCoordinate();
    }

    final ProjectedCoordinate obsPoint =
        GeoUtils.convertToEuclidean(startCoord);

    final GpsObservation initialObs =
        new GpsObservation(this.simulationName,
            this.simParameters.getStartTime(), startCoord, null,
            null, null, 0, null, obsPoint);

    this.updater =
        new VehicleStateBootstrapUpdater<GpsObservation>(
            initialObs, graph, this.simParameters.getStateParams(),
            this.rng);
    this.updater.setRandom(this.rng);
  }

  public VehicleStateDistribution<GpsObservation> computeInitialState() {

    /*
     * If the updater is null, then creation of the initial obs failed,
     * or something equally bad.
     * 
     * Otherwise, we just get the initial state from the updater.  
     * This method is effectively pointless unless we're doing something
     * special for simulations...
     * 
     */
    if (this.updater == null) {
      return null;
    }

    final VehicleStateDistribution<GpsObservation> vehicleState =
        this.updater.createInitialParticles(1).getMaxValueKey();

    return vehicleState;
  }

  public InferenceGraph getInferredGraph() {
    return this.inferredGraph;
  }

  public VehicleStateInitialParameters getInfParameters() {
    return this.infParameters;
  }

  public Random getRng() {
    return this.rng;
  }

  public long getSeed() {
    return this.seed;
  }

  public SimulationParameters getSimParameters() {
    return this.simParameters;
  }

  public String getSimulationName() {
    return this.simulationName;
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
  public Vector sampleObservation(PathState newPathState, Matrix cov) {

    final Vector groundState = newPathState.getGroundState();
    final Vector gMean =
        MotionStateEstimatorPredictor.getOg().times(groundState);

    final Matrix covSqrt = StatisticsUtil.rootOfSemiDefinite(cov);

    final Vector thisStateSample =
        MultivariateGaussian.sample(gMean, covSqrt, this.rng);
    return thisStateSample;
  }

  private VehicleStateDistribution<GpsObservation> sampleState(
    VehicleStateDistribution<GpsObservation> vehicleState, long time)
      throws NoninvertibleTransformException, TransformException {

    final VehicleStateDistribution<GpsObservation> newState =
        this.updater.update(vehicleState);

    final Matrix gCov =
        vehicleState.getObservationCovarianceParam().getValue();
    final Vector thisLoc =
        this.sampleObservation(newState.getPathStateParam()
            .getValue(), gCov);

    final Coordinate obsCoord =
        GeoUtils.convertToLatLon(thisLoc, vehicleState
            .getObservation().getObsProjected());

    final int thisRecNum =
        1 + vehicleState.getObservation().getRecordNumber();
    final GpsObservation thisObs =
        new GpsObservation(this.simulationName, new Date(time),
            obsCoord, null, null, null, thisRecNum,
            vehicleState.getObservation(), new ProjectedCoordinate(
                GeoUtils.getTransform(obsCoord),
                GeoUtils.makeCoordinate(thisLoc), obsCoord));

    newState.setObservation(thisObs);

    return newState;
  }

  public VehicleStateDistribution<GpsObservation> stepSimulation(
    VehicleStateDistribution<GpsObservation> currentState)
      throws NoninvertibleTransformException, TransformException {
    final long time =
        currentState.getObservation().getTimestamp().getTime()
            + Math.round(this.simParameters.getFrequency()) * 1000;

    /*
     * TODO: set seed for debugging
     */
    this.updater.seed = this.rng.nextLong();

    final VehicleStateDistribution<GpsObservation> vehicleState =
        this.sampleState(currentState, (long)time);

    Simulation._log.info("processed simulation observation : "
        + this.recordsProcessed + ", " + time);

    this.recordsProcessed++;
    return vehicleState;
  }

}
