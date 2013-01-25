package org.opentrackingtools.graph.paths.states;

import gov.sandia.cognition.math.matrix.Vector;

import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;

public interface PathState extends Comparable<PathState>, Cloneable {

  public abstract Vector getGroundState();

  /**
   * The state relative to its path if it's on a road, otherwise the off-road
   * state.
   * 
   * @return
   */
  public abstract Vector getGlobalState();

  /**
   * The state relative to its edge if it's on a road, otherwise the off-road
   * state.
   * 
   * @return
   */
  public abstract Vector getLocalState();

  public abstract PathEdge getEdge();

  public abstract InferredPath getPath();

  public abstract boolean isOnRoad();

  /**
   * 
   * Yields the difference between distance and velocities between this state
   * and otherState.
   * 
   * Important: The allowed movements for differencing are
   * <ul>
   * <li>
   * Both states starting at the same origin, or</li>
   * <li>
   * this state leaving from the other one's location.</li>
   * </ul>
   * Also, raw state vectors (not truncated to the path) are used.
   * 
   * @param otherState
   * @return
   */
  public abstract Vector minus(PathState otherState);

  abstract public Vector getRawState();

  public abstract PathState getTruncatedPathState();

  public abstract PathState clone();

}