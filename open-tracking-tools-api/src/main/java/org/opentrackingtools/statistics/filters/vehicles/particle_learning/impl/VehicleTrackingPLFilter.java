package org.opentrackingtools.statistics.filters.vehicles.particle_learning.impl;

import java.lang.reflect.InvocationTargetException;
import java.util.Random;

import javax.annotation.Nonnull;

import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.AbstractVTPLFilter;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.AbstractVTParticleFilterUpdater;

public class VehicleTrackingPLFilter extends
    AbstractVTPLFilter {

  public static class VehicleTrackingPLFilterUpdater extends
      AbstractVTParticleFilterUpdater {

    private static final long serialVersionUID =
        5271480648697065434L;

    public VehicleTrackingPLFilterUpdater(
      GpsObservation obs,
      InferenceGraph inferredGraph,
      VehicleStateInitialParameters parameters, 
      Random rng) throws ClassNotFoundException, SecurityException, NoSuchMethodException, IllegalArgumentException, InstantiationException, IllegalAccessException, InvocationTargetException {
      super(obs, inferredGraph, parameters, rng);
    }

    @Override
    public double computeLogLikelihood(
      VehicleState particle, GpsObservation observation) {
      return particle.getProbabilityFunction().logEvaluate(observation);
    }

    @Override
    public VehicleState update(
      VehicleState previousParameter) {
      throw new RuntimeException("Not implemented");
    }

  }

  private static final long serialVersionUID =
      -8257075186193062150L;

  public VehicleTrackingPLFilter(GpsObservation obs,
    InferenceGraph inferredGraph,
    VehicleStateInitialParameters parameters,
    @Nonnull Boolean isDebug, Random rng) throws SecurityException, IllegalArgumentException, ClassNotFoundException, NoSuchMethodException, InstantiationException, IllegalAccessException, InvocationTargetException {
    super(obs, inferredGraph, parameters,
        new VehicleTrackingPLFilterUpdater(obs,
            inferredGraph, parameters, rng), isDebug, rng);
  }
}