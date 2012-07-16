package org.openplans.tools.tracking.impl.statistics;

import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractParticleFilter;
import gov.sandia.cognition.statistics.bayesian.ParticleFilter;

import java.util.Map;
import java.util.Random;
import java.util.Set;

import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.VehicleTrackingFilter;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.util.OtpGraph;

import com.google.common.collect.Maps;
import com.google.common.collect.Sets;

public abstract class AbstractVehicleTrackingFilter extends
    AbstractParticleFilter<Observation, VehicleState> implements
    VehicleTrackingFilter<Observation, VehicleState> {

  private static final long serialVersionUID = -8257075186193062150L;

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

  public AbstractVehicleTrackingFilter(Observation obs,
    OtpGraph inferredGraph, VehicleStateInitialParameters parameters,
    ParticleFilter.Updater<Observation, VehicleState> updater,
    boolean isDebug) {
    this.isDebug = isDebug;
    this.setNumParticles(parameters.getNumParticles());
    this.inferredGraph = inferredGraph;
    this.setUpdater(updater);
    this.initialObservation = obs;
  }

  @Override
  public DataDistribution<VehicleState> createInitialLearnedObject() {
    final DataDistribution<VehicleState> dist =
        super.createInitialLearnedObject();
    if (isDebug) {
      final Set<InferredPath> evaledPaths = Sets.newHashSet();
      for (final VehicleState state : dist.getDomain()) {
        // TODO FIXME provide real info here
        evaledPaths.add(state.getPath());
      }
      this.filterInfo.put(initialObservation, new FilterInformation(
          evaledPaths, dist));
    }
    return dist;
  }

  @Override
  public FilterInformation getFilterInformation(Observation obs) {
    return this.filterInfo.get(obs);
  }

  @Override
  public Random getRandom() {
    final VehicleTrackingPathSamplerFilterUpdater updater =
        (VehicleTrackingPathSamplerFilterUpdater) this.getUpdater();
    return updater.getThreadRandom().get();
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
        prevTime == 0 ? 1d
            : (obs.getTimestamp().getTime() - prevTime) / 1000;

    if (timeDiff <= 0)
      return;

    internalUpdate(target, obs, timeDiff);

    prevTime = obs.getTimestamp().getTime();
  }

}
