package org.openplans.tools.tracking.impl.graph.paths;

import gov.sandia.cognition.math.matrix.AbstractVector;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.ObjectUtil;

import javax.annotation.Nonnull;

import org.apache.commons.lang3.builder.CompareToBuilder;
import org.openplans.tools.tracking.impl.statistics.filters.road_tracking.AbstractRoadTrackingFilter;
import org.openplans.tools.tracking.impl.statistics.filters.road_tracking.StandardRoadTrackingFilter;

import com.google.common.base.Preconditions;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.MultiLineString;
import com.vividsolutions.jts.linearref.LengthLocationMap;
import com.vividsolutions.jts.linearref.LinearLocation;

public class PathStateBelief extends AbstractPathState implements Comparable<PathStateBelief> {

  private static final long serialVersionUID = -31238492416118648L;

  private MultivariateGaussian localStateBelief;
  private MultivariateGaussian globalStateBelief;
  private MultivariateGaussian rawStateBelief;
  private MultivariateGaussian groundBelief;

  protected PathStateBelief(InferredPath path,
    MultivariateGaussian state) {
    
    this.path = path;
    this.rawStateBelief = state.clone();
    this.globalStateBelief = state.clone();
    
    /*
     * Now make sure the result is on this path.
     */
    if (!path.isEmptyPath()) {
      this.globalStateBelief.getMean().setElement(0,
          path.clampToPath(this.globalStateBelief.getMean().getElement(0)));
    } 
  }
  
  @Override
  public Vector getRawState() {
    return rawStateBelief.getMean();
  }
  
  public MultivariateGaussian getRawStateBelief() {
    return rawStateBelief;
  }

  @Override
  public PathStateBelief clone() {
    final PathStateBelief clone = (PathStateBelief) super.clone();
    clone.rawStateBelief = ObjectUtil.cloneSmart(this.rawStateBelief);
    clone.localStateBelief = ObjectUtil.cloneSmart(this.localStateBelief);
    clone.globalStateBelief = ObjectUtil.cloneSmart(this.globalStateBelief);
    clone.groundBelief = ObjectUtil.cloneSmart(this.groundBelief);
    return clone;
  }

  public Matrix getCovariance() {
    return globalStateBelief.getCovariance();
  }

  @Override
  public PathEdge getEdge() {
    if (edge == null) {
      this.edge =
          path.isEmptyPath() ? PathEdge.getEmptyPathEdge() : path
              .getEdgeForDistance(
                  globalStateBelief.getMean().getElement(0), false);
    }
    return this.edge;
  }

  public MultivariateGaussian getGroundBelief() {
    if (this.groundBelief == null) {
      final MultivariateGaussian newBelief = this.globalStateBelief.clone();
      AbstractRoadTrackingFilter.convertToGroundBelief(newBelief,
          this.getEdge(), true);
      this.groundBelief = newBelief;
      return newBelief;
    } else {
      return this.groundBelief;
    }
  }

  @Override
  public Vector getGroundState() {
    return getGroundBelief().getMean();
  }

  @Override
  public Vector getGlobalState() {
    return globalStateBelief.getMean();
  }
  
  @Override
  public Vector getLocalState() {
    return getLocalStateBelief().getMean();
  }

  public MultivariateGaussian getGlobalStateBelief() {
    return globalStateBelief;
  }
  
  public MultivariateGaussian getLocalStateBelief() {
    if (this.localStateBelief != null)
      return this.localStateBelief;
    if (this.path.isEmptyPath()) {
      this.localStateBelief = this.globalStateBelief;
    } else {
      final Vector mean = 
          Preconditions.checkNotNull(
            this.getEdge().getCheckedStateOnEdge(
                this.globalStateBelief.getMean(), 
                StandardRoadTrackingFilter.getEdgeLengthErrorTolerance(), 
                true));
      this.localStateBelief = new MultivariateGaussian(
          mean, this.globalStateBelief.getCovariance());
    }
    return this.localStateBelief; 
  }

  public static PathStateBelief getPathStateBelief(
    @Nonnull InferredPath path, @Nonnull MultivariateGaussian state) {
    Preconditions.checkArgument(!path.isEmptyPath()
        || state.getInputDimensionality() == 4);
    return new PathStateBelief(path, state);
  }

  public static PathStateBelief getPathStateBelief(
    PathState newPathState, Matrix covariance) {
    Preconditions.checkState(newPathState.getRawState().getDimensionality()
        == covariance.getNumColumns());
    PathStateBelief result = getPathStateBelief(newPathState.path, 
        new MultivariateGaussian(newPathState.getRawState(), covariance));
    result.localStateBelief = new MultivariateGaussian(newPathState.getLocalState(), covariance);
    return result;
  }

  @Override
  public int compareTo(PathStateBelief o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(this.path, o.path);
    comparator.append(this.rawStateBelief, o.rawStateBelief);
    return comparator.toComparison();
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = super.hashCode();
    result =
        prime
            * result
            + ((rawStateBelief == null) ? 0 : rawStateBelief
                .hashCode());
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (!super.equals(obj)) {
      return false;
    }
    if (getClass() != obj.getClass()) {
      return false;
    }
    PathStateBelief other = (PathStateBelief) obj;
    if (rawStateBelief == null) {
      if (other.rawStateBelief != null) {
        return false;
      }
    } else if (!rawStateBelief.equals(other.rawStateBelief)) {
      return false;
    }
    return true;
  }
}
