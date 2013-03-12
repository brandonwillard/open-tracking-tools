package org.opentrackingtools.statistics.filters.vehicles.impl;

import gov.sandia.cognition.statistics.DataDistribution;

import java.util.Collections;
import java.util.Set;

import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.impl.VehicleState;

import com.google.common.base.Objects;

public class FilterInformation {

  private final Set<InferredPath> evaluatedPaths;
  private final DataDistribution<VehicleState> resampleDist;

  @SuppressWarnings("unchecked")
  public FilterInformation(
    Set<InferredPath> evaledPaths,
    DataDistribution<VehicleState> resampleDist) {
    this.evaluatedPaths =
        (Set<InferredPath>) Objects.firstNonNull(
            evaledPaths, Collections.emptySet());
    this.resampleDist = resampleDist;
  }

  public Set<InferredPath> getEvaluatedPaths() {
    return evaluatedPaths;
  }

  public DataDistribution<VehicleState> getResampleDist() {
    return resampleDist;
  }


}
