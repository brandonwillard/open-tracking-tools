package org.openplans.tools.tracking.impl.statistics.filters.particle_learning;

import gov.sandia.cognition.statistics.bayesian.AbstractParticleFilter;

import java.util.Random;

import javax.annotation.Nonnull;

import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.statistics.filters.road_tracking.AbstractRoadTrackingFilter;
import org.openplans.tools.tracking.impl.statistics.filters.road_tracking.ErrorEstimatingRoadTrackingFilter;
import org.openplans.tools.tracking.impl.util.OtpGraph;

public class VTErrorEstimatingPLFilter extends AbstractVTPLFilter {

  private static final long serialVersionUID = -8257075186193062150L;
  
  public static class VTErrorEstimatingPLFilterUpdater extends AbstractVTParticleFilterUpdater {

    private static final long serialVersionUID =
        -1283253299558265844L;

    public VTErrorEstimatingPLFilterUpdater(Observation obs,
      OtpGraph inferredGraph, VehicleStateInitialParameters parameters, 
      Random rng) {
      super(obs, inferredGraph, parameters);
    }

    @Override
    public VehicleState update(VehicleState previousParameter) {
      return null;
    }

    @Override
    public double computeLogLikelihood(VehicleState particle,
      Observation observation) {
      return 0;
    }

    @Override
    @Nonnull
    protected AbstractRoadTrackingFilter
        createRoadTrackingFilter() {
      return new ErrorEstimatingRoadTrackingFilter(
            parameters.getObsCov(), parameters.getObsCovDof(),
            parameters.getOffRoadStateCov(), parameters.getOffRoadCovDof(),
            parameters.getOnRoadStateCov(), parameters.getOnRoadCovDof(),
            parameters.getInitialObsFreq(), this.random);
    }
    
  }

  public VTErrorEstimatingPLFilter(Observation obs,
    OtpGraph inferredGraph, VehicleStateInitialParameters parameters,
    @Nonnull Boolean isDebug) {
    super(obs, inferredGraph, parameters,
        new VTErrorEstimatingPLFilterUpdater(obs, inferredGraph,
            parameters, null), isDebug);
  }
}