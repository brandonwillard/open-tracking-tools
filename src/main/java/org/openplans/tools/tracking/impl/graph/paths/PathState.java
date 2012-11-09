package org.openplans.tools.tracking.impl.graph.paths;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import javax.annotation.Nonnull;

import com.google.common.base.Preconditions;

public class PathState extends AbstractCloneableSerializable {
  
  private static final long serialVersionUID = 2846671162796173049L;
  protected InferredPath path;
  protected Vector state;
  protected PathEdge edge;

  protected PathState(InferredPath path, Vector state) {
    this.path = path;
    this.state = state;
  }

  public PathEdge getEdge() {
    if (edge == null) {
      this.edge = path.getEdgeForDistance(state.getElement(0), false);
    }
    return this.edge;
  }

  public InferredPath getPath() {
    return path;
  }

  public Vector getState() {
    return state;
  }

  @Override
  public String toString() {
    final StringBuilder builder = new StringBuilder();
    builder.append("PathState [path=").append(path)
        .append(", state=").append(state).append("]");
    return builder.toString();
  }

  public static PathState getPathState(@Nonnull InferredPath path,
    @Nonnull Vector state) {
    Preconditions.checkArgument(!path.isEmptyPath()
        || state.getDimensionality() == 4);
    Preconditions.checkArgument(path.isEmptyPath()
        || path.isOnPath(state.getElement(0)));
    return new PathState(path, state);
  }
  
  public boolean isOnRoad() {
    return !this.path.isEmptyPath();
  }

  @Override
  public PathState clone() {
    PathState clone = (PathState) super.clone();
    clone.edge = this.edge;
    clone.path = this.path;
    clone.state = this.state.clone();
    
    return clone;
  }
}
