package org.opentrackingtools;

import gov.sandia.cognition.math.MutableDouble;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractParticleFilter;

import java.util.Map.Entry;
import java.util.Random;
import java.util.Set;

import org.opentrackingtools.distributions.CountedDataDistribution;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.updater.VehicleTrackingBootstrapUpdater;
import org.opentrackingtools.util.model.MutableDoubleCount;

import com.google.common.collect.HashMultimap;
import com.google.common.collect.Sets;

public class VehicleTrackingBootstrapFilter<O extends GpsObservation> extends
    AbstractParticleFilter<O, VehicleState<O>> {

  private static final long serialVersionUID =
      -2642221321938929247L;
  protected final InferenceGraph inferredGraph;
  protected final Boolean isDebug;

  public VehicleTrackingBootstrapFilter(O obs,
    InferenceGraph inferredGraph,
    VehicleStateInitialParameters parameters,
    Boolean isDebug, Random rng) {
    this.inferredGraph = inferredGraph;
    this.isDebug = isDebug;
    this.setUpdater(new VehicleTrackingBootstrapUpdater<O>(obs,
        inferredGraph, parameters, rng));
  }

  @Override
  public void update(
    DataDistribution<VehicleState<O>> target, O obs) {
    
    /*
     * Get predictive states
     */
    CountedDataDistribution<VehicleState<O>> resampler = new CountedDataDistribution<VehicleState<O>>(true);
    int totalCount = 0;
    final Set<Path> evaledPaths = Sets.newHashSet();
    for (Entry<VehicleState<O>, ? extends Number> entry : target.asMap().entrySet()) {

      final int count;
      if (target instanceof CountedDataDistribution<?>) {
        count = ((MutableDoubleCount) entry.getValue()).count;
      } else {
        count = 1;
      }
      totalCount += count;
      for (int i = 0; i < count; i++) {
        final VehicleState<O> predictedState = new VehicleState<O>(entry.getKey());
        predictedState.setObservation(obs);
        this.updater.update(predictedState);

        if (this.isDebug)
          evaledPaths.add(predictedState.getPathStateParam().getValue()
              .getPath());

        /*
         * Previous particle weight times new state likelihood
         */
        final double totalLogLik =
            target.getProbabilityFunction().logEvaluate(
                entry.getKey())
                + this.updater.computeLogLikelihood(predictedState, obs);

        resampler.increment(predictedState, totalLogLik, 1);
      }
    }

    assert totalCount == this.numParticles;

    final double efps =
        this.computeEffectiveParticles(resampler);

    target.clear();
    if (efps < this.numParticles * 0.9d) {
      target.incrementAll(resampler.sample(this.random, numParticles));
    } else {
      for (Entry<VehicleState<O>, MutableDouble> entry : resampler.asMap().entrySet()) {
        MutableDoubleCount value = ((MutableDoubleCount) entry.getValue());
        if (target instanceof CountedDataDistribution<?>) {
          ((CountedDataDistribution<VehicleState<O>>)target).set(
              entry.getKey(), 
              value.value, 
              value.count);
        } else {
          for (int i = 0; i < value.count; i++) {
            target.set(entry.getKey(), value.value);
          }
        }
      }
    }

    assert (target instanceof CountedDataDistribution<?>) ? 
        (((CountedDataDistribution<VehicleState<O>>) target).getTotalCount() 
            == this.numParticles) : true;
  }

}
