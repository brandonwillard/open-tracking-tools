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
  
  public final boolean isSimulation;
  
  private VehicleTrackingFilter filter;

  private final long prevTime = 0l;

  private DataDistribution<VehicleState> belief;
  private VehicleState bestState;

  private final InitialParameters initialParameters;
  private static InferredGraph inferredGraph = new InferredGraph(Api.getGraph());

  public InferenceInstance(String vehicleId, boolean isSimulation) {
    this.initialParameters = new InitialParameters(
        VectorFactory.getDefault().createVector2D(VehicleState.getGvariance(), VehicleState.getGvariance()),
        VectorFactory.getDefault().createVector2D(VehicleState.getDvariance(), VehicleState.getVvariance()),
        VectorFactory.getDefault().createVector2D(VehicleState.getDvariance(), VehicleState.getVvariance()),
        VectorFactory.getDefault().createVector2D(0.05d, 1d),
        VectorFactory.getDefault().createVector2D(1d, 0.05d)
        );
    this.vehicleId = vehicleId;
    this.isSimulation = isSimulation;
  }
  
  public InferenceInstance(String vehicleId, boolean isSimulation, InitialParameters parameters) {
    this.initialParameters = parameters;
    this.vehicleId = vehicleId;
    this.isSimulation = isSimulation;
  }

  public VehicleState getBestState() {
    return bestState;
  }

  public long getPrevTime() {
    return prevTime;
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
    recordsProcessed++;
    
  }

  private void updateFilter(Observation obs) {

    if (filter == null || belief == null) {
      filter = new VehicleTrackingFilter(obs, inferredGraph, initialParameters);
      belief = filter.createInitialLearnedObject();
    } else {
      filter.update(belief, obs);
    }

    if (belief != null)
      this.bestState = belief.getMaxValueKey();
  }

}
