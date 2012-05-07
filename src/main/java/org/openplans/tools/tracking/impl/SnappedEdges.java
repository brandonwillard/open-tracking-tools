package org.openplans.tools.tracking.impl;

import java.util.List;
import java.util.Set;

import org.opentripplanner.routing.graph.Edge;

import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableSet;

public class SnappedEdges {

  final private ImmutableSet<Edge> snappedEdges;
  final private ImmutableList<Edge> pathTraversed;

  public SnappedEdges(ImmutableSet<Edge> snappedEdges, ImmutableList<Edge> pathTraversed) {
    Preconditions.checkNotNull(snappedEdges);
    Preconditions.checkNotNull(pathTraversed);
    this.snappedEdges = snappedEdges;
    this.pathTraversed = pathTraversed;
  }

  public Set<Edge> getSnappedEdges() {
    return snappedEdges;
  }

  public List<Edge> getPathTraversed() {
    return pathTraversed;
  }

  @Override
  protected Object clone() throws CloneNotSupportedException {
    return super.clone();
  }

  
}
