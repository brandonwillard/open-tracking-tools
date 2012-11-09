package org.openplans.tools.tracking.impl.graph.paths;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import javax.annotation.Nonnull;

import com.google.common.base.Preconditions;

public class PathStateBelief  extends PathState {

  private static final long serialVersionUID = -31238492416118648L;
  
  private MultivariateGaussian stateBelief;

  protected PathStateBelief(InferredPath path,
    MultivariateGaussian state) {
    super(path, state.getMean());
    this.stateBelief = state;
  }

  public MultivariateGaussian getStateBelief() {
    return stateBelief;
  }

  public static PathStateBelief getPathStateBelief(
    @Nonnull InferredPath path, @Nonnull MultivariateGaussian state) {
    Preconditions.checkArgument(!path.isEmptyPath()
        || state.getInputDimensionality() == 4);
    Preconditions.checkArgument(path.isEmptyPath()
        || path.isOnPath(state.getMean().getElement(0)));
    return new PathStateBelief(path, state);
  }


  @Override
  public PathStateBelief clone() {
    PathStateBelief clone = (PathStateBelief) super.clone();
    clone.stateBelief = this.stateBelief.clone();
    clone.state = clone.stateBelief.getMean();
    
    return clone;
  }

  public Vector getMean() {
    return stateBelief.getMean();
  }
  
  public Matrix getCovariance() {
    return stateBelief.getCovariance();
  }

  public Double getDistanceBetween(PathStateBelief belief) {
    return null;
  }

  public Vector getGroundMean() {
    return null;
  }

  public MultivariateGaussian getGroundBelief() {
    return null;
  }
}
