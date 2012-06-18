package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.statistics.DataDistribution;

import java.util.Collection;
import java.util.Collections;

import com.google.common.base.Objects;

public class FilterInformation {

  private final InferredPath path;
  private final Collection<InferredPathEntry> evaluatedPaths;
  private final DataDistribution<VehicleState> resampleDist;

  @SuppressWarnings("unchecked")
  public FilterInformation(InferredPath path,
    Collection<InferredPathEntry> evaluatedPaths, DataDistribution<VehicleState> resampleDist) {
    this.path = path;
    this.evaluatedPaths = (Collection<InferredPathEntry>) Objects
        .firstNonNull(evaluatedPaths, Collections.emptyList());
    this.resampleDist = resampleDist;
  }

  public Collection<InferredPathEntry> getEvaluatedPaths() {
    return evaluatedPaths;
  }

  public InferredPath getPath() {
    return path;
  }

  public DataDistribution<VehicleState> getResampleDist() {
    return resampleDist;
  }

}
