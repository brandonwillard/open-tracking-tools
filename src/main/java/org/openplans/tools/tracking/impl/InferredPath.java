package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;

import com.google.common.collect.ImmutableList;

public class InferredPath {
  private final MultivariateGaussian belief; 
  private final Standard2DTrackingFilter filter;
  private final ImmutableList<InferredEdge> path;
  
  public InferredPath(ImmutableList<InferredEdge> path,
    MultivariateGaussian belief, Standard2DTrackingFilter filter) {
    this.belief = belief;
    this.filter = filter;
    this.path = path;
  }

  public MultivariateGaussian getBelief() {
    return belief;
  }

  public Standard2DTrackingFilter getFilter() {
    return filter;
  }

  public ImmutableList<InferredEdge> getPath() {
    return path;
  }

}
