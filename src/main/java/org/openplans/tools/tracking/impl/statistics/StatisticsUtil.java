package org.openplans.tools.tracking.impl.statistics;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.distribution.DefaultDataDistribution;
import gov.sandia.cognition.util.DefaultWeightedValue;

import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import com.google.common.collect.Lists;

public class StatisticsUtil {

  public static <SupportType extends Comparable> DataDistribution<SupportType> getLogNormalizedDistribution(
    List<DefaultWeightedValue<SupportType>> collection) {

    /*-
     * Normalize to avoid zero probs.
     */
    double totalLikelihood = Double.NEGATIVE_INFINITY;
    for (final DefaultWeightedValue<SupportType> weight : collection) {
      totalLikelihood = LogMath.add(
          weight.getWeight(), totalLikelihood);
    }
    
    if (totalLikelihood == Double.NEGATIVE_INFINITY)
      return null;

    final DataDistribution<SupportType> result = new DefaultDataDistribution<SupportType>();

    /*
     * Sort before putting in the data distribution
     */
    // Collections.sort(collection, new
    // Comparator<DefaultWeightedValue<SupportType>> () {
    //
    // @Override
    // public int compare(DefaultWeightedValue<SupportType> arg0,
    // DefaultWeightedValue<SupportType> arg1) {
    // return arg0.getValue().compareTo(arg1.getValue());
    // }
    //
    // });

    for (final DefaultWeightedValue<SupportType> entry : collection) {
      if (entry.getWeight() == Double.NEGATIVE_INFINITY
          || totalLikelihood == Double.NEGATIVE_INFINITY)
        continue;
      final double weight = entry.getWeight() - totalLikelihood;
      result.set(entry.getValue(), Math.exp(weight));
    }

    return result;
  }

  public static <DistributionType, SupportType extends Comparable> DataDistribution<SupportType> getLogNormalizedDistribution(
    Map<SupportType, DefaultWeightedValue<DistributionType>> map) {

    /*-
     * Normalize to avoid zero probs.
     */
    double totalLikelihood = Double.NEGATIVE_INFINITY;
    for (final DefaultWeightedValue<DistributionType> weight : map
        .values()) {
      totalLikelihood = LogMath.add(
          weight.getWeight(), totalLikelihood);
    }
    
    if (totalLikelihood == Double.NEGATIVE_INFINITY)
      return null;

    /*
     * Sort before putting in the data distribution
     */
    final List<Entry<SupportType, DefaultWeightedValue<DistributionType>>> entryList = Lists
        .newArrayList(map.entrySet());
    // Collections.sort(entryList, new Comparator<Entry<SupportType,
    // DefaultWeightedValue<DistributionType>>> () {
    //
    // @Override
    // public int compare(Entry<SupportType,
    // DefaultWeightedValue<DistributionType>> arg0,
    // Entry<SupportType, DefaultWeightedValue<DistributionType>> arg1) {
    // return arg0.getKey().compareTo(arg1.getKey());
    // }
    //
    // });

    final DataDistribution<SupportType> result = new DefaultDataDistribution<SupportType>();
    for (final Entry<SupportType, DefaultWeightedValue<DistributionType>> entry : entryList) {
      if (entry.getValue().getWeight() == Double.NEGATIVE_INFINITY)
        continue;
      final double weight = entry.getValue().getWeight()
          - totalLikelihood;
      result.set(entry.getKey(), Math.exp(weight));
    }

    return result;
  }
}
