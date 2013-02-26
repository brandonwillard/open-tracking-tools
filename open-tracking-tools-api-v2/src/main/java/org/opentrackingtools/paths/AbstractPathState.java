package org.opentrackingtools.paths;

import gov.sandia.cognition.math.matrix.AbstractVector;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.util.AbstractCloneableSerializable;
import gov.sandia.cognition.util.ObjectUtil;


public abstract class AbstractPathState extends
    AbstractCloneableSerializable implements PathState {

  private static final long serialVersionUID =
      -5009638378510031502L;

  protected InferredPath path;

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.PathState#getGroundState()
   */
  @Override
  public abstract Vector getGroundState();

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.PathState#getGlobalState()
   */
  @Override
  public abstract Vector getGlobalState();

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.PathState#getLocalState()
   */
  @Override
  public abstract Vector getLocalState();

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.PathState#getEdge()
   */
  @Override
  public abstract PathEdge getEdge();

  protected PathEdge edge;

  public AbstractPathState() {
    super();
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.PathState#getPath()
   */
  @Override
  public InferredPath getPath() {
    return path;
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.PathState#isOnRoad()
   */
  @Override
  public boolean isOnRoad() {
    return !this.path.isNullPath();
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.PathState#minus(org.opentrackingtools.graph.paths.states.AbstractPathState)
   */
  @Override
  public Vector minus(PathState fromState) {
    return PathUtils.stateDiff(fromState, this, false);
  }
  
  @Override
  public AbstractPathState clone() {
    final AbstractPathState clone =
        (AbstractPathState) super.clone();
    clone.path = ObjectUtil.cloneSmart(this.path);
    /*
     * The edge should always refer to the edge in this path,
     * so if the path gets cloned and that clone makes a clone
     * of the path edges, then we need to find the new edge.
     */
    clone.edge = null;
    return clone;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime * result
            + ((path == null) ? 0 : path.hashCode());
    result =
        prime
            * result
            + ((getGlobalState() == null) ? 0
                : ((AbstractVector) getGlobalState())
                    .hashCode());
    result =
        prime
            * result
            + ((getRawState() == null) ? 0
                : ((AbstractVector) getRawState())
                    .hashCode());
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (obj == null) {
      return false;
    }
    if (getClass() != obj.getClass()) {
      return false;
    }
    final AbstractPathState other = (AbstractPathState) obj;
    if (path == null) {
      if (other.path != null) {
        return false;
      }
    } else if (!path.equals(other.path)) {
      return false;
    }

    if (getGlobalState() == null) {
      if (other.getGlobalState() != null) {
        return false;
      }
    } else if (!((AbstractVector) getGlobalState())
        .equals((other.getGlobalState()))) {
      return false;
    }
    
    if (getRawState() == null) {
      if (other.getRawState() != null) {
        return false;
      }
    } else if (!((AbstractVector) getRawState())
        .equals((other.getRawState()))) {
      return false;
    }
    return true;
  }

  @Override
  public String toString() {
    final StringBuilder builder = new StringBuilder();
    builder.append("AbstractPathState [path=").append(path)
        .append(", state=").append(this.getGlobalState())
        .append("]");
    return builder.toString();
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.PathState#getRawState()
   */
  @Override
  abstract public Vector getRawState();

}