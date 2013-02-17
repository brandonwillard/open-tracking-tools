package org.opentrackingtools.graph.otp.impl;

import java.util.HashMap;
import java.util.List;

import gov.sandia.cognition.statistics.bayesian.BayesianCredibleInterval;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;

import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.impl.SimpleInferredPath;
import org.opentrackingtools.statistics.impl.DataCube;

import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.Iterables;

public class OtpInferredPath extends SimpleInferredPath {
  
  private static OtpInferredPath nullPath = new OtpInferredPath();
  
  private OtpInferredPath() {
    super();
  }
  
  public static InferredPath getNullPath() {
    return nullPath;
  }
  
  public static OtpInferredPath getInferredPath(
    List<? extends PathEdge> newEdges, boolean isBackward) {
    if (newEdges.size() == 1) {
      final PathEdge edge = Iterables.getOnlyElement(newEdges);
      if (edge.isNullEdge())
        return nullPath;
    }
    return new OtpInferredPath(ImmutableList.copyOf(newEdges),
        isBackward);
  }

  private OtpInferredPath(ImmutableList<PathEdge> edges,
    boolean isBackward) {
    super(edges, isBackward);
  }
  
  protected OtpInferredPath(PathEdge edge) {
    super(edge);
  }
  
  public static OtpInferredPath getInferredPath(
    PathEdge pathEdge) {
    if (pathEdge.isNullEdge())
      return nullPath;
    else
      return new OtpInferredPath(pathEdge);
  }

  @Override
  public void updateEdges(GpsObservation obs,
    MultivariateGaussian stateBelief, InferenceGraph graph) {
    
    super.updateEdges(obs, stateBelief, graph);
    
    Preconditions.checkArgument(graph instanceof OtpGraph);
    
    if (this.isNullPath())
      return;

    final BayesianCredibleInterval ciInterval =
        BayesianCredibleInterval.compute(
            new UnivariateGaussian(stateBelief.getMean()
                .getElement(1), stateBelief.getCovariance()
                .getElement(1, 1)), 0.95);

    /*
     * If we could be stopped, then don't update this
     */
    if (ciInterval.withinInterval(0d))
      return;

    for (final PathEdge edge : this.getPathEdges()) {
      edge.getInferredEdge().update(stateBelief);
      
      if (!edge.isNullEdge()) {

        final double velocity =
            stateBelief.getMean().getElement(1);
        final HashMap<String, String> attributes =
            new HashMap<String, String>();

        final Integer interval =
            Math.round(((obs.getTimestamp().getHours() * 60) + obs
                .getTimestamp().getMinutes())
                / DataCube.INTERVAL);

        attributes.put("interval", interval.toString());
        attributes.put("edge", edge.getInferredEdge()
            .getEdgeId());

        ((OtpGraph)graph).getDataCube().store(Math.abs(velocity),
            attributes);
      }
    }
  }

}
