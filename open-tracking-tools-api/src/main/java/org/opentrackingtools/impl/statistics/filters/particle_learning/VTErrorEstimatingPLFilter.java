package org.opentrackingtools.impl.statistics.filters.particle_learning;

import java.util.Random;

import javax.annotation.Nonnull;

import org.opentrackingtools.impl.Observation;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleState.VehicleStateInitialParameters;
import org.opentrackingtools.impl.statistics.filters.road_tracking.AbstractRoadTrackingFilter;
import org.opentrackingtools.impl.statistics.filters.road_tracking.ErrorEstimatingRoadTrackingFilter;
import org.opentrackingtools.util.OtpGraph;

public class VTErrorEstimatingPLFilter extends
    AbstractVTPLFilter {

  public static class VTErrorEstimatingPLFilterUpdater
      extends AbstractVTParticleFilterUpdater {

    private static final long serialVersionUID =
        -1283253299558265844L;

    public VTErrorEstimatingPLFilterUpdater(
      Observation obs, OtpGraph inferredGraph,
      VehicleStateInitialParameters parameters, Random rng) {
      super(obs, inferredGraph, parameters);
    }

    @Override
    public double computeLogLikelihood(
      VehicleState particle, Observation observation) {
      return 0;
    }

    @Override
    @Nonnull
    protected AbstractRoadTrackingFilter
        createRoadTrackingFilter() {
      return new ErrorEstimatingRoadTrackingFilter(
          parameters.getObsCov(),
          parameters.getObsCovDof(),
          parameters.getOffRoadStateCov(),
          parameters.getOffRoadCovDof(),
          parameters.getOnRoadStateCov(),
          parameters.getOnRoadCovDof(),
          parameters.getInitialObsFreq(), this.random);
    }

    @Override
    public VehicleState update(
      VehicleState previousParameter) {
      return null;
    }

  }

  private static final long serialVersionUID =
      -8257075186193062150L;

  public VTErrorEstimatingPLFilter(Observation obs,
    OtpGraph inferredGraph,
    VehicleStateInitialParameters parameters,
    @Nonnull Boolean isDebug) {
    super(obs, inferredGraph, parameters,
        new VTErrorEstimatingPLFilterUpdater(obs,
            inferredGraph, parameters, null), isDebug);
  }
}