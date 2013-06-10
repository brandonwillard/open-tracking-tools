package org.opentrackingtools.util;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.List;

import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;

public class PathEdgeNode implements Comparable<PathEdgeNode> {

  private MultivariateGaussian edgeDistribution;
  private double edgeLogLikelihood = 0d;
  private MultivariateGaussian edgeObsDistribution;

  private double obsLogLikelihood = 0d;
  final private PathEdgeNode parent;
  final private PathEdge pathEdge;
  private double transitionLogLikelihood;
  private Integer chainLength = null;

  public PathEdgeNode(PathEdge pathEdge, PathEdgeNode parent) {
    this.pathEdge = pathEdge;
    this.parent = parent;
  }

  @Override
  public int compareTo(PathEdgeNode o) {
    return Double.compare(this.getEdgeTotalLogLikelihood(),
        o.getEdgeTotalLogLikelihood());
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (obj == null) {
      return false;
    }
    if (!(obj instanceof PathEdgeNode)) {
      return false;
    }
    final PathEdgeNode other = (PathEdgeNode) obj;
    if (this.pathEdge == null) {
      if (other.pathEdge != null) {
        return false;
      }
    } else if (!this.pathEdge.equals(other.pathEdge)) {
      return false;
    }
    return true;
  }

  public MultivariateGaussian getEdgeDistribution() {
    return this.edgeDistribution;
  }

  public double getEdgeLogLikelihood() {
    return this.edgeLogLikelihood;
  }

  public MultivariateGaussian getEdgeObsDistribution() {
    return this.edgeObsDistribution;
  }

  public double getEdgeTotalLogLikelihood() {
    final double result =
        this.obsLogLikelihood + this.edgeLogLikelihood
            + this.transitionLogLikelihood;
    Preconditions.checkState(!Double.isNaN(result));
    return result;
  }

  public double getObsLogLikelihood() {
    return this.obsLogLikelihood;
  }

  public PathEdgeNode getParent() {
    return this.parent;
  }

  public Path getPath() {
    final List<PathEdge> edges = Lists.newArrayList();
    PathEdgeNode currentEdgeNode = this;
    while (currentEdgeNode != null) {
      edges.add(currentEdgeNode.getPathEdge());
      currentEdgeNode = currentEdgeNode.parent;
    }
    final Path result = new Path(Lists.reverse(edges), false);
    return result;
  }

  public PathEdge getPathEdge() {
    return this.pathEdge;
  }

  /**
   * Gets the node chain's average edge transition likelihood.
   */
  public double getTransitionLogLikelihood() {
    return this.transitionLogLikelihood;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime
            * result
            + ((this.pathEdge == null) ? 0 : this.pathEdge.hashCode());
    return result;
  }

  public void setEdgeDistribution(
    MultivariateGaussian neighborEdgePathState) {
    this.edgeDistribution = neighborEdgePathState;
  }

  public void setEdgeLogLikelihood(double g) {
    this.edgeLogLikelihood = g;
  }

  public void setEdgeObsDistribution(
    MultivariateGaussian neighborEdgeObsDist) {
    this.edgeObsDistribution = neighborEdgeObsDist;
  }

  public void setObsLogLikelihood(double h) {
    this.obsLogLikelihood = h;
  }

  /**
   * The length of the node chain (inclusive).
   * @return
   */
  public int getChainLength() {
    if (this.chainLength == null) {
      if (this.parent != null) {
        this.chainLength = this.parent.getChainLength() + 1;
      } else {
        this.chainLength = 1;
      }
    }
    return this.chainLength;
  }

  /**
   * Sets the node chain's average edge transition likelihood.
   * @param transitionLogLikelihood
   */
  public void setTransitionLogLikelihood(
    double transitionLogLikelihood) {
    final int chainLength = this.getChainLength();
    if (chainLength > 2) {
      this.transitionLogLikelihood = LogMath.add(this.parent.transitionLogLikelihood, transitionLogLikelihood)
          - Math.log(chainLength - 1);
    } else {
      this.transitionLogLikelihood = transitionLogLikelihood;
    }
  }

}
