package org.opentrackingtools.impl.statistics.filters;

import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractParticleFilter;

import java.util.Map;
import java.util.Random;
import java.util.Set;

import javax.annotation.Nonnull;

import org.opentrackingtools.impl.Observation;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleState.VehicleStateInitialParameters;
import org.opentrackingtools.impl.graph.paths.InferredPath;
import org.opentrackingtools.impl.statistics.filters.particle_learning.AbstractVTParticleFilterUpdater;
import org.opentrackingtools.util.OtpGraph;

import com.google.common.collect.Maps;
import com.google.common.collect.Sets;

public abstract class AbstractVehicleTrackingFilter extends
    AbstractParticleFilter<Observation, VehicleState>
    implements
    VehicleTrackingFilter<Observation, VehicleState> {

  private static final long serialVersionUID =
      -8257075186193062150L;

  /*
   * Populate this when you want a generalized graph. TODO Otherwise, one is
   * created for each particle.
   */
  protected final OtpGraph inferredGraph;

  protected double prevTime = 0;

  protected DataDistribution<VehicleState> previousResampleDist;

  protected final Map<Observation, FilterInformation> filterInfo =
      Maps.newHashMap();

  protected final boolean isDebug;

  protected final Observation initialObservation;

  protected final VehicleStateInitialParameters parameters;

  public AbstractVehicleTrackingFilter(Observation obs,
    OtpGraph inferredGraph,
    VehicleStateInitialParameters parameters,
    AbstractVTParticleFilterUpdater updater,
    @Nonnull Boolean isDebug) {
    this.parameters = parameters;
    this.isDebug = isDebug;
    this.setNumParticles(parameters.getNumParticles());
    this.inferredGraph = inferredGraph;
    this.setUpdater(updater);
    this.initialObservation = obs;
    this.random = new Random(parameters.getSeed());
    updater.setRandom(random);
  }

  public AbstractVehicleTrackingFilter(Observation obs,
    OtpGraph inferredGraph,
    VehicleStateInitialParameters parameters,
    AbstractVTParticleFilterUpdater updater,
    @Nonnull Boolean isDebug, Random rng) {
    this.parameters = parameters;
    this.isDebug = isDebug;
    this.setNumParticles(parameters.getNumParticles());
    this.inferredGraph = inferredGraph;
    this.setUpdater(updater);
    this.initialObservation = obs;
    this.random = rng;
    updater.setRandom(random);
  }

  @Nonnull
  @Override
  public DataDistribution<VehicleState>
      createInitialLearnedObject() {
    final DataDistribution<VehicleState> dist =
        super.createInitialLearnedObject();
    if (isDebug) {
      final Set<InferredPath> evaledPaths =
          Sets.newHashSet();
      for (final VehicleState state : dist.getDomain()) {
        // TODO FIXME provide real info here
        evaledPaths.add(state.getBelief().getPath());
      }
      this.filterInfo.put(initialObservation,
          new FilterInformation(evaledPaths, dist, null));
    }

    this.prevTime =
        this.initialObservation.getTimestamp().getTime();

    return dist;
  }

  @Override
  public FilterInformation getFilterInformation(
    Observation obs) {
    return this.filterInfo.get(obs);
  }

  @Override
  public double getLastProcessedTime() {
    return prevTime;
  }

  @Nonnull
  @Override
  public Random getRandom() {
    return this.random;
  }

  protected abstract void internalUpdate(
    DataDistribution<VehicleState> target, Observation obs,
    double timeDiff);

  /**
   * Note: this skips observations with a time delta of zero or less.
   */
  @Override
  public void update(DataDistribution<VehicleState> target,
    Observation obs) {

    final double timeDiff =
        prevTime == 0d
            ? 0d
            : (obs.getTimestamp().getTime() - prevTime) / 1000d;

    if (timeDiff <= 0d)
      return;

    internalUpdate(target, obs, timeDiff);

    prevTime = obs.getTimestamp().getTime();
  }

}
