package models;

import org.openplans.tools.tracking.impl.InferredGraph;
import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.Standard2DTrackingFilter;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.VehicleTrackingFilter;

import gov.sandia.cognition.math.UnivariateStatisticsUtil;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;


import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;

import controllers.Api;
import play.data.validation.Constraints.*;

/**
 * This class holds inference data for a particular vehicle
 * 
 * @author bwillard
 * 
 */
public class InferenceInstance {

  @Required
  final public String vehicleId;
  
  public int recordsProcessed = 0;
  
  private VehicleTrackingFilter filter;

  private final long prevTime = 0l;

  private DataDistribution<VehicleState> belief;
  private VehicleState bestState;
  private static InferredGraph inferredGraph = new InferredGraph(Api.getGraph());

  public InferenceInstance(String vehicleId) {
    this.vehicleId = vehicleId;
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
      filter = new VehicleTrackingFilter(obs, inferredGraph);
      belief = filter.createInitialLearnedObject();
    } else {
      filter.update(belief, obs);
    }

    if (belief != null)
      this.bestState = belief.getMaxValueKey();
  }

}
