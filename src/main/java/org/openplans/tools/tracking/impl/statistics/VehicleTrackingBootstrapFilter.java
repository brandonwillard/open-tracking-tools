package org.openplans.tools.tracking.impl.statistics;

import gov.sandia.cognition.statistics.DataDistribution;

import java.util.List;
import java.util.Set;

import org.openplans.tools.tracking.impl.LogDefaultDataDistribution;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.VehicleState.VehicleStateInitialParameters;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.util.OtpGraph;

import com.google.common.collect.HashMultimap;
import com.google.common.collect.Lists;
import com.google.common.collect.Sets;

public class VehicleTrackingBootstrapFilter extends
    AbstractVehicleTrackingFilter {

  private static final long serialVersionUID = -2642221321938929247L;

  public VehicleTrackingBootstrapFilter(Observation obs,
    OtpGraph inferredGraph, VehicleStateInitialParameters parameters,
    boolean isDebug) {
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
      state.getMovementFilter().setCurrentTimeDiff(timeDiff);

      final int count =
          ((LogDefaultDataDistribution<VehicleState>) target)
              .getCount(state);
      totalCount += count;
      for (int i = 0; i < count; i++) {

        final VehicleState predictedState =
            ((VehicleTrackingPathSamplerFilterUpdater) this.updater)
                .update(state, obs);

        evaledPaths.add(state.getPath());

        final double totalLogLik =
            predictedState.getProbabilityFunction().logEvaluate(obs);

        resampler.add(new WrappedWeightedValue<VehicleState>(
            predictedState, totalLogLik, 1));
      }
    }

    assert totalCount == this.numParticles;

    final LogDefaultDataDistribution<VehicleState> prePosteriorDist =
        StatisticsUtil.getLogNormalizedDistribution(resampler);

    final double efps =
        this.computeEffectiveParticles(prePosteriorDist);

    final DataDistribution<VehicleState> posteriorDist;
    if (efps < this.numParticles * 0.9d) {
      // TODO low variance resampling?
      final DataDistribution<VehicleState> resampleDist =
          new LogDefaultDataDistribution<VehicleState>(
              prePosteriorDist.sample(
                  ((VehicleTrackingPathSamplerFilterUpdater) updater)
                      .getThreadRandom().get(), numParticles));
      posteriorDist = resampleDist;
    } else {
      posteriorDist = prePosteriorDist;
    }

    target.clear();
    ((LogDefaultDataDistribution<VehicleState>) target)
        .copyAll(posteriorDist);

    assert ((LogDefaultDataDistribution<VehicleState>) target)
        .getTotalCount() == this.numParticles;

    this.filterInfo.put(obs, new FilterInformation(evaledPaths,
        prePosteriorDist));

  }

}
