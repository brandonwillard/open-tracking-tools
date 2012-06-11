package org.openplans.tools.tracking.impl;

import java.util.Collection;
import java.util.Collections;

import com.google.common.base.Objects;

public class FilterInformation {

  private final InferredPath path;
  private final Collection<InferredPathEntry> evaluatedPaths;

  @SuppressWarnings("unchecked")
  public FilterInformation(InferredPath path,
    Collection<InferredPathEntry> evaluatedPaths) {
    this.path = path;
    this.evaluatedPaths = (Collection<InferredPathEntry>) Objects
        .firstNonNull(evaluatedPaths, Collections.emptyList());
  }

  public Collection<InferredPathEntry> getEvaluatedPaths() {
    return evaluatedPaths;
  }

  public InferredPath getPath() {
    return path;
  }

}
