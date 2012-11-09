package org.openplans.tools.tracking.impl.statistics.filters.particle_learning;

import javax.annotation.Nonnull;

import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.util.OtpGraph;

public class VehicleTrackingPLFilter extends AbstractVTPLFilter {

  public static class VehicleTrackingPLFilterUpdater extends
      VehicleTrackingParticleFilterUpdater {

    private static final long serialVersionUID = 5271480648697065434L;

    public VehicleTrackingPLFilterUpdater(Observation obs,
      OtpGraph inferredGraph, VehicleStateInitialParameters parameters) {
      super(obs, inferredGraph, parameters);
    }

    @Override
    public VehicleState update(VehicleState previousParameter) {
      // TODO Auto-generated method stub
      return null;
    }

    @Override
    public double computeLogLikelihood(VehicleState particle,
      Observation observation) {
      // TODO Auto-generated method stub
      return 0;
    }

  }

  private static final long serialVersionUID = -8257075186193062150L;

  public VehicleTrackingPLFilter(Observation obs,
    OtpGraph inferredGraph, VehicleStateInitialParameters parameters,
    @Nonnull Boolean isDebug) {
    super(obs, inferredGraph, parameters,
        new VehicleTrackingPLFilterUpdater(obs, inferredGraph,
            parameters), isDebug);
  }
}