package org.openplans.tools.tracking.impl.graph.paths;

import gov.sandia.cognition.math.matrix.AbstractVector;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.util.ObjectUtil;

import javax.annotation.Nonnull;

import org.apache.commons.lang3.builder.CompareToBuilder;
import org.openplans.tools.tracking.impl.statistics.filters.road_tracking.AbstractRoadTrackingFilter;
import org.openplans.tools.tracking.impl.statistics.filters.road_tracking.StandardRoadTrackingFilter;

import com.google.common.base.Preconditions;

public class PathState extends AbstractPathState implements Comparable<PathState> {
  
  private static final long serialVersionUID = 2846671162796173049L;
  private Vector globalState;
  private Vector groundState;
  private Vector localState;
  private Vector rawState;

  protected PathState(InferredPath path, Vector state) {
    
    this.path = path;
    this.rawState = state.clone();
    this.globalState = state.clone();
    /*
     * Now make sure the result is on this path.
     */
    if (!path.isEmptyPath()) {
      this.globalState.setElement(0,
          path.clampToPath(this.globalState.getElement(0)));
    } 
  }

  @Override
  public Vector getGlobalState() {
    return globalState;
  }
  
  @Override
  public Vector getLocalState() {
    if (this.localState != null)
      return this.localState;
    if (this.path.isEmptyPath()) {
      this.localState = this.globalState;
    } else {
      this.localState = this.getEdge().getCheckedStateOnEdge(
          this.globalState, 
          StandardRoadTrackingFilter.getEdgeLengthErrorTolerance(),
          true);
    }
    return this.localState;
  }

  @Override
  public PathEdge getEdge() {
    if (edge == null) {
      this.edge = path.isEmptyPath() ? 
          PathEdge.getEmptyPathEdge() : 
            path.getEdgeForDistance(globalState.getElement(0), false);
    }
    return Preconditions.checkNotNull(this.edge);
  }

  @Override
  public String toString() {
    final StringBuilder builder = new StringBuilder();
    builder.append("PathState [path=").append(path)
        .append(", state=").append(globalState).append("]");
    return builder.toString();
  }

  public static PathState getPathState(@Nonnull InferredPath path,
    @Nonnull Vector state) {
    Preconditions.checkArgument(!path.isEmptyPath()
        || state.getDimensionality() == 4);
    return new PathState(path, state);
  }
  
  @Override
  public Vector getGroundState() {
    if (this.groundState != null)
      return this.groundState;
    
    this.groundState = AbstractRoadTrackingFilter.convertToGroundState(
        this.globalState, this.getEdge(), true);
    
    return this.groundState;
  }
  
  @Override
  public PathState clone() {
    PathState clone = (PathState) super.clone();
    clone.rawState = ObjectUtil.cloneSmart(this.rawState);
    clone.localState = ObjectUtil.cloneSmart(this.localState);
    clone.globalState = ObjectUtil.cloneSmart(this.globalState);
    clone.groundState = ObjectUtil.cloneSmart(this.groundState);
    return clone;
  }

  @Override
  public int compareTo(PathState o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(this.path, o.path);
    comparator.append((AbstractVector)this.globalState, (AbstractVector)o.globalState);
    comparator.append((AbstractVector)this.rawState, 
        (AbstractVector)o.rawState);
    return comparator.toComparison();
  }

  @Override
  public Vector getRawState() {
    return rawState;
  }
  
}
