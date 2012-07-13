package org.openplans.tools.tracking.impl.statistics;

import gov.sandia.cognition.collection.ScalarMap.Entry;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractParticleFilter;

import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import org.openplans.tools.tracking.impl.LogDefaultDataDistribution;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.VehicleStateConditionalParams;
import org.openplans.tools.tracking.impl.VehicleTrackingFilter;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.util.OtpGraph;

import com.google.common.collect.HashMultimap;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Sets;

public class VehicleTrackingBootstrapFilter extends
    AbstractParticleFilter<Observation, VehicleState> implements
    VehicleTrackingFilter<Observation, VehicleState> {

  private static final long serialVersionUID = -2642221321938929247L;

  /*
   * Populate this when you want a generalized graph. TODO Otherwise, one is
   * created for each particle.
   */
  private final OtpGraph inferredGraph;

  private double prevTime = 0;

  private DataDistribution<VehicleState> previousResampleDist;

  private final Map<Observation, FilterInformation> filterInfo = Maps
      .newHashMap();

  private final boolean isDebug;

  private final Observation initialObservation;

  public VehicleTrackingBootstrapFilter(Observation obs,
    OtpGraph inferredGraph, VehicleStateInitialParameters parameters,
    boolean isDebug) {
    this.isDebug = isDebug;
    this.setNumParticles(50);
    this.inferredGraph = inferredGraph;
    VehicleTrackingPathSamplerFilterUpdater updater = new VehicleTrackingPathSamplerFilterUpdater(
        obs, this.inferredGraph, parameters);
    this.setUpdater(updater);
    this.initialObservation = obs;
//    this.setRandom(updater.getThreadRandom().get());
  }

  @Override
  public DataDistribution<VehicleState> createInitialLearnedObject() {
    final DataDistribution<VehicleState> dist = super
        .createInitialLearnedObject();
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
    final VehicleTrackingPathSamplerFilterUpdater updater = (VehicleTrackingPathSamplerFilterUpdater) this
        .getUpdater();
    return updater.getThreadRandom().get();
  }

  /**
   * Note: this skips observations with a time delta of zero or less.
   */
  @Override
  public void update(DataDistribution<VehicleState> target,
    Observation obs) {

    final double timeDiff = prevTime == 0 ? 1d : (obs.getTimestamp()
        .getTime() - prevTime) / 1000;

    if (timeDiff <= 0)
      return;

    HashMultimap.create();
    Sets.newHashSet();

    /*
     * Get predictive states
     */
    final List<WrappedWeightedValue<VehicleState>> resampler = Lists
        .newArrayList();
    int totalCount = 0;
    Set<InferredPath> evaledPaths = Sets.newHashSet();
    for (final VehicleState state : target.getDomain()) {
      state.getMovementFilter().setCurrentTimeDiff(timeDiff);
      
      final int count = ((LogDefaultDataDistribution<VehicleState>)target).getCount(state);
      totalCount += count;
      for (int i = 0; i < count; i++) {
  
        final VehicleState predictedState = ((VehicleTrackingPathSamplerFilterUpdater) this.updater)
            .update(state, obs);
        
        evaledPaths.add(state.getPath());
        
        final PathEdge currentPathEdge = PathEdge.getEdge(predictedState
            .getInferredEdge());
  
        final VehicleStateConditionalParams edgeLoc = new VehicleStateConditionalParams(
            currentPathEdge, obs.getProjectedPoint());
        final double totalLogLik = predictedState.getProbabilityFunction()
            .logEvaluate(edgeLoc);
  
        resampler.add(new WrappedWeightedValue<VehicleState>(
            predictedState, totalLogLik, 1));
      }
    }

    assert totalCount == this.numParticles;
    
    final DataDistribution<VehicleState> prePosteriorDist = StatisticsUtil
        .getLogNormalizedDistribution(resampler);
    
    final double efps = this.computeEffectiveParticles(prePosteriorDist);
    
    final DataDistribution<VehicleState> posteriorDist;
    if (efps < this.numParticles * 3d/4d) {
      // TODO low variance resampling?
      final DataDistribution<VehicleState> resampleDist = 
          new LogDefaultDataDistribution<VehicleState>(prePosteriorDist.sample(
              ((VehicleTrackingPathSamplerFilterUpdater) updater).getThreadRandom().get(), 
              numParticles));
      posteriorDist = resampleDist;
    } else {
      posteriorDist = prePosteriorDist;
    }

    target.clear();
    ((LogDefaultDataDistribution<VehicleState>)target).copyAll(posteriorDist);
    
    assert ((LogDefaultDataDistribution<VehicleState>)target).getTotalCount() == this.numParticles;
    
    this.filterInfo.put(obs, new FilterInformation(
        evaledPaths, prePosteriorDist));
    
    prevTime = obs.getTimestamp().getTime();
  }

}
