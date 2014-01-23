package org.opentrackingtools;

import gov.sandia.cognition.math.MutableDouble;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractParticleFilter;

import java.util.Map.Entry;
import java.util.Random;
import java.util.Set;

import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.updater.VehicleStateBootstrapUpdater;
import org.opentrackingtools.util.model.MutableDoubleCount;

import com.google.common.collect.Sets;
import com.statslibextensions.statistics.distribution.CountedDataDistribution;

public class VehicleStateBootstrapFilter<O extends GpsObservation>
    extends AbstractParticleFilter<O, VehicleStateDistribution<O>> {

  private static final long serialVersionUID = -2642221321938929247L;
  protected final InferenceGraph inferredGraph;
  protected final Boolean isDebug;

  public VehicleStateBootstrapFilter(O obs,
    InferenceGraph inferredGraph,
    VehicleStateInitialParameters parameters, Boolean isDebug,
    Random rng) {
    this.inferredGraph = inferredGraph;
    this.isDebug = isDebug;
    this.setUpdater(new VehicleStateBootstrapUpdater<O>(obs,
        inferredGraph, parameters, rng));
  }

  @Override
  public void update(
    DataDistribution<VehicleStateDistribution<O>> target, O obs) {

    /*
     * Get predictive states
     */
    final CountedDataDistribution<VehicleStateDistribution<O>> resampler =
        new CountedDataDistribution<VehicleStateDistribution<O>>(true);
    int totalCount = 0;
    final Set<Path> evaledPaths = Sets.newHashSet();
    for (final Entry<VehicleStateDistribution<O>, ? extends Number> entry : target
        .asMap().entrySet()) {

      final int count;
      if (target instanceof CountedDataDistribution<?>) {
        count = ((MutableDoubleCount) entry.getValue()).count;
      } else {
        count = 1;
      }
      totalCount += count;
      for (int i = 0; i < count; i++) {
        final VehicleStateDistribution<O> predictedState =
            entry.getKey().clone();
        predictedState.setObservation(obs);
        this.updater.update(predictedState);

        if (this.isDebug) {
          evaledPaths.add(predictedState.getPathStateParam()
              .getValue().getPath());
        }

        /*
         * Previous particle weight times new state likelihood
         */
        final double totalLogLik =
            target.getProbabilityFunction().logEvaluate(
                entry.getKey())
                + this.updater.computeLogLikelihood(predictedState,
                    obs);

        resampler.increment(predictedState, totalLogLik, 1);
      }
    }

    assert totalCount == this.numParticles;

    final double efps = this.computeEffectiveParticles(resampler);

    target.clear();
    if (efps < this.numParticles * 0.9d) {
      target.incrementAll(resampler.sample(this.random,
          this.numParticles));
    } else {
      for (final Entry<VehicleStateDistribution<O>, MutableDouble> entry : resampler
          .asMap().entrySet()) {
        final MutableDoubleCount value =
            ((MutableDoubleCount) entry.getValue());
        if (target instanceof CountedDataDistribution<?>) {
          ((CountedDataDistribution<VehicleStateDistribution<O>>) target)
              .set(entry.getKey(), value.value, value.count);
        } else {
          for (int i = 0; i < value.count; i++) {
            target.set(entry.getKey(), value.value);
          }
        }
      }
    }

    assert (target instanceof CountedDataDistribution<?>)
        ? (((CountedDataDistribution<VehicleStateDistribution<O>>) target)
            .getTotalCount() == this.numParticles) : true;
  }

}
