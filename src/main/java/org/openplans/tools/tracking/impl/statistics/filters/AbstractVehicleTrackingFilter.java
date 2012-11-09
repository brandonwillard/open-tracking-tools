package org.openplans.tools.tracking.impl.statistics.filters;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractParticleFilter;
import gov.sandia.cognition.statistics.distribution.DefaultDataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import javax.annotation.Nonnull;

import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.InferredPathEntry;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.graph.paths.PathStateBelief;
import org.openplans.tools.tracking.impl.statistics.DefaultCountedDataDistribution;
import org.openplans.tools.tracking.impl.statistics.OnOffEdgeTransDirMulti;
import org.openplans.tools.tracking.impl.statistics.filters.particle_learning.VehicleTrackingParticleFilterUpdater;
import org.openplans.tools.tracking.impl.statistics.filters.road_tracking.StandardRoadTrackingFilter;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.routing.edgetype.StreetEdge;
import org.opentripplanner.routing.graph.Edge;

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

  protected final VehicleStateInitialParameters parameters;

  public AbstractVehicleTrackingFilter(Observation obs,
    OtpGraph inferredGraph, VehicleStateInitialParameters parameters,
    VehicleTrackingParticleFilterUpdater updater,
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

  @Override
  public DataDistribution<VehicleState> createInitialLearnedObject() {
    final DataDistribution<VehicleState> dist =
        super.createInitialLearnedObject();
    if (isDebug) {
      final Set<InferredPath> evaledPaths = Sets.newHashSet();
      for (final VehicleState state : dist.getDomain()) {
        // TODO FIXME provide real info here
        evaledPaths.add(state.getBelief().getPath());
      }
      this.filterInfo.put(initialObservation, new FilterInformation(
          evaledPaths, dist));
    }

    this.prevTime = this.initialObservation.getTimestamp().getTime();

    return dist;
  }

  @Override
  public FilterInformation getFilterInformation(Observation obs) {
    return this.filterInfo.get(obs);
  }

  @Override
  public double getLastProcessedTime() {
    return prevTime;
  }

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
        prevTime == 0d ? 0d
            : (obs.getTimestamp().getTime() - prevTime) / 1000d;

    if (timeDiff <= 0d)
      return;

    internalUpdate(target, obs, timeDiff);

    prevTime = obs.getTimestamp().getTime();
  }
  
}
