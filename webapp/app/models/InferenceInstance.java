package models;

import org.openplans.tools.tracking.impl.InferredGraph;
import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.Standard2DTrackingFilter;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.VehicleState.InitialParameters;
import org.openplans.tools.tracking.impl.VehicleTrackingFilter;

import gov.sandia.cognition.math.UnivariateStatisticsUtil;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;


import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;

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

  private DataDistribution<VehicleState> belief;
  private VehicleState bestState;

  private final InitialParameters initialParameters;

  public int totalRecords = 0;
  
  private static InferredGraph inferredGraph = new InferredGraph(Api.getGraph());

  public InferenceInstance(String vehicleId, boolean isSimulation) {
    this.initialParameters = new InitialParameters(
        VectorFactory.getDefault().createVector2D(VehicleState.getGvariance(), VehicleState.getGvariance()),
        VectorFactory.getDefault().createVector2D(VehicleState.getDvariance(), VehicleState.getVvariance()),
        VectorFactory.getDefault().createVector2D(VehicleState.getDvariance(), VehicleState.getVvariance()),
        VectorFactory.getDefault().createVector2D(0.05d, 1d),
        VectorFactory.getDefault().createVector2D(1d, 0.05d),
        0l);
    this.vehicleId = vehicleId;
    this.isSimulation = isSimulation;
  }
  
  public InferenceInstance(String vehicleId, boolean isSimulation, InitialParameters parameters) {
    this.initialParameters = parameters;
    this.vehicleId = vehicleId;
    this.isSimulation = isSimulation;
    this.simSeed = parameters.getSeed();
  }

  public VehicleState getBestState() {
    return bestState;
  }

  public DataDistribution<VehicleState> getStateBelief() {
    return belief;
  }

  public String getVehicleId() {
    return vehicleId;
  }

  /**
   * Update the tracking filter and the graph's edge-velocity distributions.
   * 
   * @param record
   */
  public void update(Observation obs) {
    updateFilter(obs);
  }

  private void updateFilter(Observation obs) {

    if (filter == null || belief == null) {
      filter = new VehicleTrackingFilter(obs, inferredGraph, initialParameters);
      filter.getRandom().setSeed(simSeed);
      belief = filter.createInitialLearnedObject();
    } else {
      filter.update(belief, obs);
    }

    if (belief != null)
      this.bestState = belief.getMaxValueKey();
  }

  public int getRecordsProcessed() {
    return recordsProcessed;
  }

  public long getSimSeed() {
    return simSeed;
  }

  public boolean isSimulation() {
    return isSimulation;
  }

  public VehicleTrackingFilter getFilter() {
    return filter;
  }

  public DataDistribution<VehicleState> getBelief() {
    return belief;
  }

  public InitialParameters getInitialParameters() {
    return initialParameters;
  }

  public int getTotalRecords() {
    return totalRecords;
  }

  public static InferredGraph getInferredGraph() {
    return inferredGraph;
  }

}
