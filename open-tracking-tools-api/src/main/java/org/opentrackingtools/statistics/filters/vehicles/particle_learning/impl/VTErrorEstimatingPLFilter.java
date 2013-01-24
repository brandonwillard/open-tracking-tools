package org.opentrackingtools.statistics.filters.vehicles.particle_learning.impl;

import java.util.Random;

import javax.annotation.Nonnull;

import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleState.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.AbstractVTPLFilter;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.AbstractVTParticleFilterUpdater;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.ErrorEstimatingRoadTrackingFilter;


public class VTErrorEstimatingPLFilter extends
    AbstractVTPLFilter {

  public static class VTErrorEstimatingPLFilterUpdater
      extends AbstractVTParticleFilterUpdater {

    private static final long serialVersionUID =
        -1283253299558265844L;

    public VTErrorEstimatingPLFilterUpdater(
      GpsObservation obs, InferenceGraph inferredGraph,
      VehicleStateInitialParameters parameters, Random rng) {
      super(obs, inferredGraph, parameters);
    }

    @Override
    public double computeLogLikelihood(
      VehicleState particle, GpsObservation observationFactory) {
      return 0;
    }

    @Override
    @Nonnull
    protected AbstractRoadTrackingFilter
        createRoadTrackingFilter() {
      return new ErrorEstimatingRoadTrackingFilter(
          inferenceGraph, parameters.getObsCov(),
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

  public VTErrorEstimatingPLFilter(GpsObservation obs,
    InferenceGraph inferredGraph,
    VehicleStateInitialParameters parameters,
    @Nonnull Boolean isDebug) {
    super(obs, inferredGraph, parameters,
        new VTErrorEstimatingPLFilterUpdater(obs,
            inferredGraph, parameters, null), isDebug);
  }
}