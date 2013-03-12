package org.opentrackingtools.statistics.filters.vehicles;

import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractParticleFilter;

import java.util.Map;
import java.util.Random;
import java.util.Set;

import javax.annotation.Nonnull;

import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.statistics.filters.vehicles.impl.FilterInformation;
import org.opentrackingtools.statistics.filters.vehicles.particle_learning.AbstractVTParticleFilterUpdater;

import com.google.common.collect.Maps;
import com.google.common.collect.Sets;

public abstract class AbstractVehicleTrackingFilter extends
    AbstractParticleFilter<GpsObservation, VehicleState>
    implements
    VehicleTrackingFilter<GpsObservation, VehicleState> {

  private static final long serialVersionUID =
      -8257075186193062150L;

  /*
   * Populate this when you want a generalized graph. TODO Otherwise, one is
   * created for each particle.
   */
  protected final InferenceGraph inferredGraph;

  protected Long prevTime = null;

  protected DataDistribution<VehicleState> previousResampleDist;

  protected final Map<GpsObservation, FilterInformation> filterInfo =
      Maps.newHashMap();

  protected final boolean isDebug;

  protected final GpsObservation initialObservation;

  protected final VehicleStateInitialParameters parameters;

  public AbstractVehicleTrackingFilter(GpsObservation obs,
    InferenceGraph inferredGraph,
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
          new FilterInformation(evaledPaths, dist));
    }

    this.prevTime =
        this.initialObservation.getTimestamp().getTime();

    return dist;
  }

  @Override
  public FilterInformation getFilterInformation(
    GpsObservation obs) {
    return this.filterInfo.get(obs);
  }

  @Override
  public Long getLastProcessedTime() {
    return prevTime;
  }

  @Nonnull
  @Override
  public Random getRandom() {
    return this.random;
  }

  protected abstract void internalUpdate(
    DataDistribution<VehicleState> target, GpsObservation obs,
    double timeDiff);

  /**
   * Note: this skips observations with a time delta of zero or less.
   * The filter must be initialized by calling {@link #createInitialLearnedObject()}
   */
  @Override
  public void update(DataDistribution<VehicleState> target,
    GpsObservation obs) {

    final double timeDiff =
        prevTime == null 
            ? 0d
            : (obs.getTimestamp().getTime() - prevTime) / 1000d;

    if (timeDiff <= 0d)
      return;

    internalUpdate(target, obs, timeDiff);

    prevTime = obs.getTimestamp().getTime();
  }

}
