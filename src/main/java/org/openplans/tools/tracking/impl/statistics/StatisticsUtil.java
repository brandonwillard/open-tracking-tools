package org.openplans.tools.tracking.impl.statistics;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.distribution.DefaultDataDistribution;
import gov.sandia.cognition.util.DefaultWeightedValue;

import java.util.Collection;
import java.util.Map;
import java.util.Map.Entry;

public class StatisticsUtil {

  public static <SupportType> DataDistribution<SupportType> getLogNormalizedDistribution(
    Collection<DefaultWeightedValue<SupportType>> collection) {
    final DataDistribution<SupportType> result = new DefaultDataDistribution<SupportType>();

    /*-
     * Normalize to avoid zero probs.
     */
    double totalLikelihood = Double.NEGATIVE_INFINITY;
    for (final DefaultWeightedValue<SupportType> weight : collection) {
      totalLikelihood = LogMath.add(
          weight.getWeight(), totalLikelihood);
    }

    for (final DefaultWeightedValue<SupportType> entry : collection) {
      final double weight = entry.getWeight() - totalLikelihood;
      result.set(entry.getValue(), Math.exp(weight));
    }

    return result;
  }

  public static <DistributionType, SupportType> DataDistribution<SupportType> getLogNormalizedDistribution(
    Map<SupportType, DefaultWeightedValue<DistributionType>> map) {
    final DataDistribution<SupportType> result = new DefaultDataDistribution<SupportType>();

    /*-
     * Normalize to avoid zero probs.
     */
    double totalLikelihood = Double.NEGATIVE_INFINITY;
    for (final DefaultWeightedValue<DistributionType> weight : map
        .values()) {
      totalLikelihood = LogMath.add(
          weight.getWeight(), totalLikelihood);
    }

    for (final Entry<SupportType, DefaultWeightedValue<DistributionType>> entry : map
        .entrySet()) {
      final double weight = entry.getValue().getWeight()
          - totalLikelihood;
      result.set(entry.getKey(), Math.exp(weight));
    }

    return result;
  }
}
