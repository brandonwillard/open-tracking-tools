package org.opentrackingtools.util;

import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;

import no.uib.cipr.matrix.sparse.Preconditioner;

import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;

import java.util.List;

public class PathEdgeNode implements Comparable<PathEdgeNode> {
  
  private double obsLogLikelihood = 0d;
  private double edgeLogLikelihood = 0d;
  private double pathToEdgeTotalLogLikelihood = 0d;
  
  final private PathEdge pathEdge;
  final private PathEdgeNode parent;
  private MultivariateGaussian edgeDistribution;
  private MultivariateGaussian edgeObsDistribution;
  private double transitionLogLikelihood;
  
  public PathEdgeNode(PathEdge pathEdge, PathEdgeNode parent) {
    this.pathEdge = pathEdge;
    this.parent = parent;
  }
  

  @Override
  public int compareTo(PathEdgeNode o) {
    return Double.compare(this.getEdgeTotalLogLikelihood(), o.getEdgeTotalLogLikelihood());
  }


  public double getEdgeTotalLogLikelihood() {
    final double result = obsLogLikelihood + edgeLogLikelihood + transitionLogLikelihood;
    Preconditions.checkState(!Double.isNaN(result));
    return result;
  }

  public double getObsLogLikelihood() {
    return obsLogLikelihood;
  }

  public void setObsLogLikelihood(double h) {
    this.obsLogLikelihood = h;
  }

  public double getEdgeLogLikelihood() {
    return edgeLogLikelihood;
  }

  public void setEdgeLogLikelihood(double g) {
    this.edgeLogLikelihood = g;
  }
  
  public PathEdge getPathEdge() {
    return pathEdge;
  }


  public double getPathToEdgeTotalLogLikelihood() {
    return pathToEdgeTotalLogLikelihood;
  }


  public void setPathToEdgeTotalLogLikelihood(double f) {
    this.pathToEdgeTotalLogLikelihood = f;
  }


  public PathEdgeNode getParent() {
    return parent;
  }


  public Path getPath() {
    List<PathEdge> edges = Lists.newArrayList();
    PathEdgeNode currentEdgeNode = this;
    while (currentEdgeNode != null) {
      edges.add(currentEdgeNode.getPathEdge());
      currentEdgeNode = currentEdgeNode.parent;
    }
    final Path result = new Path(Lists.reverse(edges), false);
    return result;
  }


  public void setEdgeDistribution(MultivariateGaussian neighborEdgePathState) {
    this.edgeDistribution = neighborEdgePathState;
  }

  public void setEdgeObsDistribution(MultivariateGaussian neighborEdgeObsDist) {
    this.edgeObsDistribution = neighborEdgeObsDist;
  }


  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((pathEdge == null) ? 0 : pathEdge.hashCode());
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
    if (!(obj instanceof PathEdgeNode)) {
      return false;
    }
    PathEdgeNode other = (PathEdgeNode) obj;
    if (pathEdge == null) {
      if (other.pathEdge != null) {
        return false;
      }
    } else if (!pathEdge.equals(other.pathEdge)) {
      return false;
    }
    return true;
  }

  public MultivariateGaussian getEdgeObsDistribution() {
    return this.edgeObsDistribution;
  }

  public MultivariateGaussian getEdgeDistribution() {
    return this.edgeDistribution;
  }

  public double getTransitionLogLikelihood() {
    return this.transitionLogLikelihood;
  }
  
  public void setTransitionLogLikelihood(double transitionLogLikelihood) {
    this.transitionLogLikelihood = transitionLogLikelihood;
  }
  
}
