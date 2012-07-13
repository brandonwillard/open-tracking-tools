package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;

import java.util.List;
import java.util.Map;

import org.codehaus.jackson.annotate.JsonIgnore;

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;

public class VehicleStatePerformanceResult {

  private List<UnivariateGaussian.SufficientStatistic> stats;

  public List<Map<String, Double>> getResults() {
    final List<Map<String, Double>> result = Lists.newArrayList();
    for (final UnivariateGaussian.SufficientStatistic stat : stats) {
      final Map<String, Double> resultMap = Maps.newHashMap();
      resultMap.put("mean", stat.getMean());
      resultMap.put("variance", stat.getVariance());
      result.add(resultMap);
    }

    return result;
  }

  @JsonIgnore
  public List<UnivariateGaussian.SufficientStatistic> getStats() {
    return stats;
  }

  public void setStats(
    List<UnivariateGaussian.SufficientStatistic> stats) {
    this.stats = stats;
  }

}
