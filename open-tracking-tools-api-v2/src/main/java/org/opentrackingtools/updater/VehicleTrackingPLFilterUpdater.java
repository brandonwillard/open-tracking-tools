package org.opentrackingtools.updater;

import java.lang.reflect.InvocationTargetException;
import java.util.Random;

public class VehicleTrackingPLFilterUpdater extends
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