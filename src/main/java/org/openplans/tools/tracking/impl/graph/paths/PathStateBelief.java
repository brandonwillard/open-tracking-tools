package org.openplans.tools.tracking.impl.graph.paths;

import javax.annotation.Nonnull;


import com.google.common.base.Preconditions;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

public class PathStateBelief implements PathStateInterface {
  
  final private InferredPath path;
  final private MultivariateGaussian state;
  private PathEdge edge;
  
  protected PathStateBelief(InferredPath path, MultivariateGaussian state) {
    this.path = path;
    this.state = state;
  }

  public static PathStateBelief getPathStateBelief(@Nonnull InferredPath path, 
    @Nonnull MultivariateGaussian state) {
    Preconditions.checkArgument(!path.isEmptyPath() || state.getInputDimensionality() == 4);
    Preconditions.checkArgument(path.isEmptyPath() || path.isOnPath(state.getMean().getElement(0)));
    return new PathStateBelief(path, state);
  }
  
  @Override
  public PathEdge getEdge() {
    if (edge == null) {
      this.edge = path.getEdgeForDistance(getState().getElement(0), false);
    }
    return this.edge;
  }

  @Override
  public Vector getState() {
    return state.getMean();
  }
  
  public MultivariateGaussian getStateBelief() {
    return state;
  }

  @Override
  public InferredPath getPath() {
    return this.path;
  }

  @Override
  public String toString() {
    StringBuilder builder = new StringBuilder();
    builder.append("PathStateBelief [path=").append(path)
        .append(", state=").append(state).append("]");
    return builder.toString();
  }

}
