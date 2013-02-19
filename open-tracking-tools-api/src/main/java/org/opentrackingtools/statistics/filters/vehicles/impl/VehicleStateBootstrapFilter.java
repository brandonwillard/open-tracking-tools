package org.opentrackingtools.statistics.filters.vehicles.impl;

import gov.sandia.cognition.statistics.DataDistribution;

import java.lang.reflect.InvocationTargetException;
import java.util.List;
import java.util.Random;
import java.util.Set;

import javax.annotation.Nonnull;

import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.VehicleStateInitialParameters;
import org.opentrackingtools.impl.WrappedWeightedValue;
import org.opentrackingtools.statistics.distributions.impl.DefaultCountedDataDistribution;
import org.opentrackingtools.statistics.filters.vehicles.AbstractVehicleStateFilter;
import org.opentrackingtools.statistics.impl.StatisticsUtil;

import com.google.common.collect.HashMultimap;
import com.google.common.collect.Lists;
import com.google.common.collect.Sets;

public class VehicleStateBootstrapFilter extends
    AbstractVehicleStateFilter {

  private static final long serialVersionUID =
      -2642221321938929247L;

  public VehicleStateBootstrapFilter(GpsObservation obs,
    InferenceGraph inferredGraph,
    VehicleStateInitialParameters parameters,
    @Nonnull Boolean isDebug, Random rng) throws SecurityException, IllegalArgumentException, ClassNotFoundException, NoSuchMethodException, InstantiationException, IllegalAccessException, InvocationTargetException {
    super(obs, inferredGraph, parameters,
        new VehicleStatePathSamplerUpdater(obs,
            inferredGraph, parameters, rng), isDebug, rng);
  }

  @Override
  protected void internalUpdate(
    DataDistribution<VehicleState> target, GpsObservation obs,
    double timeDiff) {
    HashMultimap.create();
    Sets.newHashSet();

    /*
     * Get predictive states
     */
    final List<WrappedWeightedValue<VehicleState>> resampler =
        Lists.newArrayList();
    int totalCount = 0;
    final Set<InferredPath> evaledPaths = Sets.newHashSet();
    for (final VehicleState state : target.getDomain()) {
      state.getMovementFilter()
          .setCurrentTimeDiff(timeDiff);

      final int count =
          ((DefaultCountedDataDistribution<VehicleState>) target)
              .getCount(state);
      totalCount += count;
      for (int i = 0; i < count; i++) {

        final VehicleState predictedState =
            ((VehicleStatePathSamplerUpdater) this.updater)
                .update(state, obs);

        if (this.isDebug)
          evaledPaths.add(predictedState.getBelief()
              .getPath());

        /*
         * Previous particle weight times new state likelihood
         */
        final double totalLogLik =
            target.getProbabilityFunction().logEvaluate(
                state)
                + predictedState.getProbabilityFunction()
                    .logEvaluate(obs);
        //              predictedState.getMovementFilter().logLikelihood(
        //                  obs.getProjectedPoint(), predictedState.getBelief().getMean(),
        //                  PathEdge.getEdge(predictedState.getInferredEdge(), 0d, 
        //                      predictedState.getPath().getIsBackward()));

        resampler
            .add(new WrappedWeightedValue<VehicleState>(
                predictedState, totalLogLik, 1));
      }
    }

    assert totalCount == this.numParticles;

    final DefaultCountedDataDistribution<VehicleState> prePosteriorDist =
        StatisticsUtil
            .getLogNormalizedDistribution(resampler);

    final double efps =
        this.computeEffectiveParticles(prePosteriorDist);

    final DataDistribution<VehicleState> posteriorDist;
    if (efps < this.numParticles * 0.9d) {
      // TODO low variance resampling?
      final DataDistribution<VehicleState> resampleDist =
          new DefaultCountedDataDistribution<VehicleState>(
              prePosteriorDist.sample(this.random,
                  numParticles));
      posteriorDist = resampleDist;
    } else {
      /*
       * Sometimes the log normalized distribution doesn't have the exact
       * amount we need, so we need to sample the remainder. 
       */
      posteriorDist = prePosteriorDist;
      final int numDiff =
          this.numParticles
              - prePosteriorDist.getTotalCount();
      if (numDiff != 0) {
        posteriorDist.incrementAll(prePosteriorDist.sample(
            this.random, numDiff));
      }
    }

    target.clear();
    ((DefaultCountedDataDistribution<VehicleState>) target)
        .copyAll(posteriorDist);

    assert ((DefaultCountedDataDistribution<VehicleState>) target)
        .getTotalCount() == this.numParticles;

    if (this.isDebug)
      this.filterInfo.put(obs, new FilterInformation(
          evaledPaths, prePosteriorDist, null));

  }

}
