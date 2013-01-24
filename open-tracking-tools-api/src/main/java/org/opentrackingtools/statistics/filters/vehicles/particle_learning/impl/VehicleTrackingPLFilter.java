package org.opentrackingtools.statistics.filters.vehicles.particle_learning.impl;

import javax.annotation.Nonnull;

import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleState.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.AbstractVTPLFilter;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.AbstractVTParticleFilterUpdater;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.StandardRoadTrackingFilter;

public class VehicleTrackingPLFilter extends
    AbstractVTPLFilter {

  public static class VehicleTrackingPLFilterUpdater extends
      AbstractVTParticleFilterUpdater {

    private static final long serialVersionUID =
        5271480648697065434L;

    public VehicleTrackingPLFilterUpdater(GpsObservation obs,
      InferenceGraph inferredGraph,
      VehicleStateInitialParameters parameters) {
      super(obs, inferredGraph, parameters);
    }

    @Override
    public double computeLogLikelihood(
      VehicleState particle, GpsObservation observationFactory) {
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

  public VehicleTrackingPLFilter(GpsObservation obs,
    InferenceGraph inferredGraph,
    VehicleStateInitialParameters parameters,
    @Nonnull Boolean isDebug) {
    super(obs, inferredGraph, parameters,
        new VehicleTrackingPLFilterUpdater(obs,
            inferredGraph, parameters), isDebug);
  }
}