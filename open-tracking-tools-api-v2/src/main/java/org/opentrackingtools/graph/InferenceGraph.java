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

  public Collection<InferredPath> getPaths(VehicleState fromState,
    GpsObservation toCoord);

  public Collection<InferredEdge> getTopoEquivEdges(
    InferredEdge edge);

  public Envelope getGPSGraphExtent();

  public Collection<InferredEdge> getNearbyEdges(
    DistributionWithMean<Vector> tmpInitialBelief,
    AbstractRoadTrackingFilter tmpTrackingFilter);

  public Collection<InferredEdge> getNearbyEdges(Vector projLocation,
    double radius);

  public InferredEdge getNullInferredEdge();
  public InferredPath getNullPath();
  public PathEdge getNullPathEdge();

  public Collection<InferredEdge>
      getIncomingTransferableEdges(InferredEdge infEdge);
  
  public Collection<InferredEdge>
      getOutgoingTransferableEdges(InferredEdge infEdge);

  public Envelope getProjGraphExtent();

  public PathEdge getPathEdge(InferredEdge edge, double d,
    Boolean b);

  public InferredPath getInferredPath(PathEdge pathEdge);

  public InferredPath getInferredPath(
    List<PathEdge> currentPath, Boolean b);

  public boolean edgeHasReverse(Geometry edge);
  
  public InferredEdge getInferredEdge(String id);

  public VehicleState createVehicleState(
      GpsObservation obs,
      AbstractRoadTrackingFilter trackingFilter,
      PathStateBelief pathStateBelief, OnOffEdgeTransDistribution edgeTransDist,
      VehicleState parent);

}