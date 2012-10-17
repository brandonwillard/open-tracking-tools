package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;

import java.util.List;
import java.util.Map;

import org.codehaus.jackson.annotate.JsonIgnore;

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;

public class VehicleStatePerformanceResult {

  public static class SufficientStatisticRecord {

    final long time;
    final UnivariateGaussian.SufficientStatistic stat;
    
    public SufficientStatisticRecord(long time, UnivariateGaussian.SufficientStatistic stat) {
      this.time = time;
      this.stat = stat;
      
    }
    
    public long getTime() {
      return time;
    }
    
    public Double getMean() {
      return stat.getMean();
    }

    public Double getVariance() {
      return stat.getVariance();
    }

  }

  private List<SufficientStatisticRecord> stats;

  public List<Map<String, Double>> getResults() {
    final List<Map<String, Double>> result = Lists.newArrayList();
    for (final SufficientStatisticRecord stat : stats) {
      final Map<String, Double> resultMap = Maps.newHashMap();
      resultMap.put("mean", stat.getMean());
      resultMap.put("variance", stat.getVariance());
      result.add(resultMap);
    }

    return result;
  }

  @JsonIgnore
  public List<SufficientStatisticRecord> getStats() {
    return stats;
  }

  public void setStats(
    List<SufficientStatisticRecord> stats) {
    this.stats = stats;
  }

}
