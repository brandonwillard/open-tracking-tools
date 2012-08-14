package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.evaluator.Evaluator;
import gov.sandia.cognition.learning.data.InputOutputPair;
import gov.sandia.cognition.learning.data.TargetEstimatePair;
import gov.sandia.cognition.learning.performance.SupervisedPerformanceEvaluator;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Collection;
import java.util.List;

import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.statistics.filters.StandardRoadTrackingFilter;

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

    final List<UnivariateGaussian.SufficientStatistic> stats =
        Lists.newArrayList();
    for (final TargetEstimatePair<VehicleState, DataDistribution<VehicleState>> pair : data) {

      final UnivariateGaussian.SufficientStatistic stat =
          new UnivariateGaussian.SufficientStatistic();

      final MultivariateGaussian targetBelief =
          pair.getTarget().getBelief().clone();
      StandardRoadTrackingFilter.convertToGroundBelief(targetBelief,
          PathEdge.getEdge(pair.getTarget().getInferredEdge()));

      for (final VehicleState estimate : pair.getEstimate()
          .getDomain()) {

        final MultivariateGaussian estimateBelief =
            estimate.getBelief().clone();
        StandardRoadTrackingFilter.convertToGroundBelief(
            estimateBelief,
            PathEdge.getEdge(estimate.getInferredEdge()));

        final double se =
            targetBelief.getMean().euclideanDistanceSquared(
                estimateBelief.getMean());
        stat.update(se);
      }
      stats.add(stat);
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
