package org.openplans.tools.tracking.impl.statistics.filters.particle_learning;

import javax.annotation.Nonnull;

import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.util.OtpGraph;

public class VTErrorEstimatingPLFilter extends AbstractVTPLFilter {

  private static final long serialVersionUID = -8257075186193062150L;
  
  public static class VTErrorEstimatingPLFilterUpdater extends VehicleTrackingParticleFilterUpdater {

    private static final long serialVersionUID =
        -1283253299558265844L;

    public VTErrorEstimatingPLFilterUpdater(Observation obs,
      OtpGraph inferredGraph, VehicleStateInitialParameters parameters) {
      super(obs, inferredGraph, parameters);
      // TODO Auto-generated constructor stub
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

  public VTErrorEstimatingPLFilter(Observation obs,
    OtpGraph inferredGraph, VehicleStateInitialParameters parameters,
    @Nonnull Boolean isDebug) {
    super(obs, inferredGraph, parameters,
        new VTErrorEstimatingPLFilterUpdater(obs, inferredGraph,
            parameters), isDebug);
  }
}