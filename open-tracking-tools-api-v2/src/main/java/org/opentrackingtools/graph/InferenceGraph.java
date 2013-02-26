package org.opentrackingtools.graph;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.DistributionWithMean;

import java.util.Collection;
import java.util.List;

import org.opentrackingtools.distributions.OnOffEdgeTransDistribution;
import org.opentrackingtools.edges.InferredEdge;
import org.opentrackingtools.estimators.AbstractRoadTrackingFilter;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.InferredPath;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.paths.PathStateBelief;

import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;

public interface InferenceGraph {

  public abstract Collection<InferredPath> getPaths(VehicleState fromState,
    GpsObservation toCoord);

  public abstract Collection<InferredEdge> getTopoEquivEdges(
    InferredEdge edge);

  public abstract Envelope getGPSGraphExtent();

  public abstract Collection<InferredEdge> getNearbyEdges(
    DistributionWithMean<Vector> tmpInitialBelief,
    AbstractRoadTrackingFilter tmpTrackingFilter);

  public abstract Collection<InferredEdge> getNearbyEdges(Vector projLocation,
    double radius);

  public abstract InferredEdge getNullInferredEdge();
  public abstract InferredPath getNullPath();
  public abstract PathEdge getNullPathEdge();

  public abstract Collection<InferredEdge>
      getIncomingTransferableEdges(InferredEdge infEdge);
  
  public abstract Collection<InferredEdge>
      getOutgoingTransferableEdges(InferredEdge infEdge);

  public abstract Envelope getProjGraphExtent();

  public abstract PathEdge getPathEdge(InferredEdge edge, double d,
    Boolean b);

  public abstract InferredPath getInferredPath(PathEdge pathEdge);

  public abstract InferredPath getInferredPath(
    List<PathEdge> currentPath, Boolean b);

  public abstract boolean edgeHasReverse(Geometry edge);
  
  public abstract InferredEdge getInferredEdge(String id);

  public abstract VehicleState createVehicleState(
      GpsObservation obs,
      AbstractRoadTrackingFilter trackingFilter,
      PathStateBelief pathStateBelief, OnOffEdgeTransDistribution edgeTransDist,
      VehicleState parent);

}