package org.opentrackingtools.impl.statistics.filters;

import gov.sandia.cognition.statistics.DataDistribution;

import java.util.List;
import java.util.Set;

import javax.annotation.Nonnull;

import org.opentrackingtools.impl.Observation;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.impl.WrappedWeightedValue;
import org.opentrackingtools.impl.VehicleState.VehicleStateInitialParameters;
import org.opentrackingtools.impl.graph.paths.InferredPath;
import org.opentrackingtools.impl.statistics.DefaultCountedDataDistribution;
import org.opentrackingtools.impl.statistics.StatisticsUtil;
import org.opentrackingtools.util.OtpGraph;

import com.google.common.collect.HashMultimap;
import com.google.common.collect.Lists;
import com.google.common.collect.Sets;

public class VehicleTrackingBootstrapFilter extends
    AbstractVehicleTrackingFilter {

  private static final long serialVersionUID =
      -2642221321938929247L;

  public VehicleTrackingBootstrapFilter(Observation obs,
    OtpGraph inferredGraph,
    VehicleStateInitialParameters parameters,
    @Nonnull Boolean isDebug) {
    super(obs, inferredGraph, parameters,
        new VehicleTrackingPathSamplerFilterUpdater(obs,
            inferredGraph, parameters), isDebug);
  }

  @Override
  protected void internalUpdate(
    DataDistribution<VehicleState> target, Observation obs,
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
            ((VehicleTrackingPathSamplerFilterUpdater) this.updater)
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
