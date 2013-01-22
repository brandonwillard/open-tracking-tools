package org.opentrackingtools.impl.statistics.filters.particle_learning;

import javax.annotation.Nonnull;

import org.opentrackingtools.impl.Observation;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleState.VehicleStateInitialParameters;
import org.opentrackingtools.impl.statistics.filters.road_tracking.AbstractRoadTrackingFilter;
import org.opentrackingtools.impl.statistics.filters.road_tracking.StandardRoadTrackingFilter;
import org.opentrackingtools.util.OtpGraph;

public class VehicleTrackingPLFilter extends
    AbstractVTPLFilter {

  public static class VehicleTrackingPLFilterUpdater extends
      AbstractVTParticleFilterUpdater {

    private static final long serialVersionUID =
        5271480648697065434L;

    public VehicleTrackingPLFilterUpdater(Observation obs,
      OtpGraph inferredGraph,
      VehicleStateInitialParameters parameters) {
      super(obs, inferredGraph, parameters);
    }

    @Override
    public double computeLogLikelihood(
      VehicleState particle, Observation observation) {
      // TODO Auto-generated method stub
      return 0;
    }

    @Override
    @Nonnull
    protected AbstractRoadTrackingFilter
        createRoadTrackingFilter() {
      return new StandardRoadTrackingFilter(
          parameters.getObsCov(),
          parameters.getOffRoadStateCov(),
          parameters.getOnRoadStateCov(),
          parameters.getInitialObsFreq());
    }

    @Override
    public VehicleState update(
      VehicleState previousParameter) {
      // TODO Auto-generated method stub
      return null;
    }

  }

  private static final long serialVersionUID =
      -8257075186193062150L;

  public VehicleTrackingPLFilter(Observation obs,
    OtpGraph inferredGraph,
    VehicleStateInitialParameters parameters,
    @Nonnull Boolean isDebug) {
    super(obs, inferredGraph, parameters,
        new VehicleTrackingPLFilterUpdater(obs,
            inferredGraph, parameters), isDebug);
  }
}