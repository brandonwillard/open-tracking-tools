package org.openplans.tools.tracking.impl.graph.paths;

import javax.annotation.Nonnull;


import com.google.common.base.Preconditions;

import gov.sandia.cognition.math.matrix.Vector;

public class PathState implements PathStateInterface {
  final private InferredPath path;
  final private Vector state;
  private PathEdge edge;
  
  protected PathState(InferredPath path, Vector state) {
    this.path = path;
    this.state = state;
  }

  public static PathState getPathState(@Nonnull InferredPath path, 
    @Nonnull Vector state) {
    Preconditions.checkArgument(!path.isEmptyPath() || state.getDimensionality() == 4);
    Preconditions.checkArgument(path.isEmptyPath() || path.isOnPath(state.getElement(0)));
    return new PathState(path, state);
  }

  @Override
  public InferredPath getPath() {
    return path;
  }

  @Override
  public Vector getState() {
    return state;
  }

  @Override
  public PathEdge getEdge() {
    if (edge == null) {
      this.edge = path.getEdgeForDistance(state.getElement(0), false);
    }
    return this.edge;
  }
  
  @Override
  public String toString() {
    StringBuilder builder = new StringBuilder();
    builder.append("PathState [path=").append(path)
        .append(", state=").append(state).append("]");
    return builder.toString();
  }
  
}
