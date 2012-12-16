package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.evaluator.Evaluator;
import gov.sandia.cognition.learning.data.InputOutputPair;
import gov.sandia.cognition.learning.data.TargetEstimatePair;
import gov.sandia.cognition.learning.performance.SupervisedPerformanceEvaluator;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Collection;
import java.util.List;

import org.openplans.tools.tracking.impl.VehicleStatePerformanceResult.SufficientStatisticRecord;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.statistics.filters.road_tracking.AbstractRoadTrackingFilter;

import com.google.common.collect.Lists;

public class VehicleTrackingPerformanceEvaluator extends
    AbstractCloneableSerializable
    implements
    SupervisedPerformanceEvaluator<Observation, VehicleState, DataDistribution<VehicleState>, VehicleStatePerformanceResult> {

  private static final long serialVersionUID = 2188027766955238277L;

  @Override
  public VehicleTrackingPerformanceEvaluator clone() {
    final VehicleTrackingPerformanceEvaluator cloned =
        (VehicleTrackingPerformanceEvaluator) super.clone();
    return cloned;
  }

  @Override
  public
      VehicleStatePerformanceResult
      evaluatePerformance(
        Collection<? extends TargetEstimatePair<VehicleState, DataDistribution<VehicleState>>> data) {

    final List<SufficientStatisticRecord> stats =
        Lists.newArrayList();
    for (final TargetEstimatePair<VehicleState, DataDistribution<VehicleState>> pair : data) {

      final UnivariateGaussian.SufficientStatistic stat =
          new UnivariateGaussian.SufficientStatistic();

      final Vector targetBeliefMean = pair
              .getTarget().getBelief().getGroundState();

      for (final VehicleState estimate : pair.getEstimate()
          .getDomain()) {

        final Vector estimateBeliefMean = estimate
                .getBelief().getGroundState();

        final double se =
            pair.getEstimate().getProbabilityFunction()
                .evaluate(estimate)
                * targetBeliefMean
                    .euclideanDistanceSquared(estimateBeliefMean);
        stat.update(se);
      }

      final SufficientStatisticRecord statRecord =
          new SufficientStatisticRecord(pair.getFirst()
              .getObservation().getTimestamp().getTime(), stat);
      stats.add(statRecord);
    }

    final VehicleStatePerformanceResult results =
        new VehicleStatePerformanceResult();
    results.setStats(stats);
    return results;
  }

  @Override
  public
      VehicleStatePerformanceResult
      evaluatePerformance(
        Evaluator<? super Observation, DataDistribution<VehicleState>> object,
        Collection<? extends InputOutputPair<Observation, VehicleState>> data) {
    // TODO Auto-generated method stub
    return null;
  }

}
