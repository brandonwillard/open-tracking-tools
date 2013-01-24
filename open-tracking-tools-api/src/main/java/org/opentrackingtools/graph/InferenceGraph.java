package org.opentrackingtools.graph;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.Distribution;
import gov.sandia.cognition.statistics.DistributionWithMean;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.Collection;
import java.util.List;
import java.util.Set;

import org.opentrackingtools.graph.edges.InferredEdge;
import org.opentrackingtools.graph.edges.impl.SimpleInferredEdge;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.states.PathState;
import org.opentrackingtools.graph.paths.states.PathStateBelief;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentripplanner.routing.graph.Edge;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;

public interface InferenceGraph {

  public abstract Set<InferredPath> getPaths(VehicleState fromState,
    Coordinate toCoord);

  public abstract Set<InferredEdge> getTopoEquivEdges(
    InferredEdge edge);

  public abstract Envelope getGPSGraphExtent();

  public abstract Collection<InferredEdge> getNearbyEdges(
    DistributionWithMean<Vector> tmpInitialBelief,
    AbstractRoadTrackingFilter<?> tmpTrackingFilter);

  public abstract Collection<InferredEdge> getNearbyEdges(Vector projLocation,
    double radius);

  public abstract InferredEdge getNullInferredEdge();
  public abstract InferredPath getNullPath();
  public abstract PathEdge getNullPathEdge();

  public abstract List<InferredEdge>
      getIncomingTransferableEdges(InferredEdge infEdge);
  
  public abstract List<InferredEdge>
      getOutgoingTransferableEdges(InferredEdge infEdge);

  public abstract Envelope getProjGraphExtent();

  public abstract PathEdge getPathEdge(InferredEdge edge, double d,
    boolean b);

  public abstract InferredPath getInferredPath(PathEdge pathEdge);

  public abstract InferredPath getInferredPath(
    List<PathEdge> currentPath, boolean b);

  public abstract boolean edgeHasReverse(Geometry edge);
  
  public abstract InferredEdge getInferredEdge(String id);

}