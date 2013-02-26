package org.opentrackingtools.graph.paths.edges.impl;

import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineString;

import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.edges.InferredEdge;
import org.opentrackingtools.graph.paths.states.PathStateBelief;

public class MtaPathEdge extends SimplePathEdge {

  private static final long serialVersionUID = 2604670165406618124L;

  public MtaPathEdge(InferredEdge parentEdge, Geometry edge, Double distToStartOfEdge,
      Boolean isBackward) {
    super(parentEdge, edge, distToStartOfEdge, isBackward);
  }

  @Override
  public MultivariateGaussian getPriorPredictive(PathStateBelief belief,
      GpsObservation obs) {
//    final MultivariateGaussian priorPred = belief.getGlobalStateBelief().clone();
//    if (priorPred.getMean().getElement(0) < this.distToStartOfEdge) {
//      priorPred.getMean().setElement(0, this.distToStartOfEdge);
//    } else if (priorPred.getMean().getElement(0) >= this.distToStartOfEdge + this.edge.getLength()) {
//      priorPred.getMean().setElement(0, this.distToStartOfEdge);
//    }
//    return priorPred;
    return super.getPriorPredictive(belief, obs);
  }

}
