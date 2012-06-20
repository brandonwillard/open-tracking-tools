package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.statistics.DataDistribution;

import java.util.Collection;
import java.util.Collections;
import java.util.Set;

import com.google.common.base.Objects;

public class FilterInformation {

  private final Set<InferredPath> evaluatedPaths;
  private final DataDistribution<VehicleState> resampleDist;

  @SuppressWarnings("unchecked")
  public FilterInformation(
    Set<InferredPath> evaluatedPaths,
    DataDistribution<VehicleState> resampleDist) {
    this.evaluatedPaths = (Set<InferredPath>)
        Objects.firstNonNull(evaluatedPaths, Collections.emptySet());
    this.resampleDist = resampleDist;
  }

  public Set<InferredPath> getEvaluatedPaths() {
    return evaluatedPaths;
  }

  public DataDistribution<VehicleState> getResampleDist() {
    return resampleDist;
  }

}
