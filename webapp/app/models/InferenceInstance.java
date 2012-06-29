package models;

import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.DataDistribution;
import inference.InferenceResultRecord;
import inference.InferenceService.INFO_LEVEL;

import java.util.Collection;
import java.util.Collections;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.VehicleState.InitialParameters;
import org.openplans.tools.tracking.impl.statistics.FilterInformation;
import org.openplans.tools.tracking.impl.statistics.VehicleTrackingFilter;
import org.openplans.tools.tracking.impl.util.OtpGraph;

import controllers.Api;

/**
 * This class holds inference data for a particular vehicle
 * 
 * @author bwillard
 * 
 */
public class InferenceInstance {

  final public String vehicleId;

  public int recordsProcessed = 0;

  public long simSeed = 0l;

  public final boolean isSimulation;

  private VehicleTrackingFilter filter;

  private final Queue<InferenceResultRecord> resultRecords = new ConcurrentLinkedQueue<InferenceResultRecord>();

  private DataDistribution<VehicleState> postBelief;
  private DataDistribution<VehicleState> resampleBelief;
  private VehicleState bestState;

  private final InitialParameters initialParameters;

  public int totalRecords = 0;

  private final INFO_LEVEL infoLevel;

  private static OtpGraph inferredGraph = Api.getGraph();

  public InferenceInstance(String vehicleId, boolean isSimulation,
    INFO_LEVEL infoLevel) {
    this.initialParameters = new InitialParameters(
        VectorFactory.getDefault().createVector2D(
            VehicleState.getGvariance(), VehicleState.getGvariance()),
        VectorFactory.getDefault().createVector2D(
            VehicleState.getDvariance(), VehicleState.getVvariance()),
        VectorFactory.getDefault().createVector2D(
            VehicleState.getDvariance(), VehicleState.getVvariance()),
        VectorFactory.getDefault().createVector2D(0.05d, 1d),
        VectorFactory.getDefault().createVector2D(1d, 0.05d), 0l);
    this.vehicleId = vehicleId;
    this.isSimulation = isSimulation;
    this.infoLevel = infoLevel;
  }

  public InferenceInstance(String vehicleId, boolean isSimulation,
    INFO_LEVEL infoLevel, InitialParameters parameters) {
    this.initialParameters = parameters;
    this.vehicleId = vehicleId;
    this.isSimulation = isSimulation;
    this.simSeed = parameters.getSeed();
    this.infoLevel = infoLevel;
  }

  public VehicleState getBestState() {
    return bestState;
  }

  public VehicleTrackingFilter getFilter() {
    return filter;
  }

  public INFO_LEVEL getInfoLevel() {
    return infoLevel;
  }

  public InitialParameters getInitialParameters() {
    return initialParameters;
  }

  public DataDistribution<VehicleState> getPostBelief() {
    return this.postBelief;
  }

  public int getRecordsProcessed() {
    return recordsProcessed;
  }

  public DataDistribution<VehicleState> getResampleBelief() {
    return this.resampleBelief;
  }

  public Collection<InferenceResultRecord> getResultRecords() {
    return Collections.unmodifiableCollection(this.resultRecords);
  }

  public long getSimSeed() {
    return simSeed;
  }

  public int getTotalRecords() {
    return totalRecords;
  }

  public String getVehicleId() {
    return vehicleId;
  }

  public boolean isSimulation() {
    return isSimulation;
  }

  /**
   * Update the tracking filter and the graph's edge-velocity distributions.
   * 
   * @param record
   */
  public void update(Observation obs) {

    updateFilter(obs);
    this.recordsProcessed++;

    final InferenceResultRecord infResult = InferenceResultRecord
        .createInferenceResultRecord(obs, this);

    if (infoLevel == INFO_LEVEL.SINGLE_RESULT
        && !this.resultRecords.isEmpty())
      this.resultRecords.poll();

    this.resultRecords.add(infResult);
  }

  public void update(VehicleState actualState, Observation obs,
    boolean performInference) {

    if (performInference)
      updateFilter(obs);

    this.recordsProcessed++;

    final InferenceResultRecord result = InferenceResultRecord
        .createInferenceResultRecord(
            obs, this, actualState, postBelief.getMaxValueKey(),
            postBelief.clone(),
            resampleBelief != null ? resampleBelief.clone() : null);

    if (infoLevel == INFO_LEVEL.SINGLE_RESULT
        && !this.resultRecords.isEmpty())
      this.resultRecords.poll();

    this.resultRecords.add(result);
  }

  private void updateFilter(Observation obs) {

    if (filter == null || postBelief == null) {
      filter = new VehicleTrackingFilter(
          obs, inferredGraph, initialParameters,
          infoLevel.compareTo(INFO_LEVEL.DEBUG) >= 0);
      filter.getRandom().setSeed(simSeed);
      postBelief = filter.createInitialLearnedObject();
    } else {
      filter.update(postBelief, obs);
      if (infoLevel.compareTo(INFO_LEVEL.DEBUG) >= 0) {
        final FilterInformation filterInfo = filter
            .getFilterInformation(obs);
        resampleBelief = filterInfo != null ? filterInfo
            .getResampleDist() : null;
      }
    }

    if (postBelief != null)
      this.bestState = postBelief.getMaxValueKey();
  }

  public static OtpGraph getInferredGraph() {
    return inferredGraph;
  }

}
