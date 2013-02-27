package org.opentrackingtools.graph;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.DistributionWithMean;

import java.util.Collection;
import java.util.List;

import org.opentrackingtools.distributions.OnOffEdgeTransDistribution;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.estimators.AbstractRoadTrackingFilter;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;

import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;

public interface InferenceGraph {

  public Collection<Path> getPaths(VehicleState fromState,
    GpsObservation toCoord);

  public Collection<InferenceGraphEdge> getTopoEquivEdges(
    InferenceGraphEdge edge);

  public Envelope getGPSGraphExtent();

  public Collection<InferenceGraphEdge> getNearbyEdges(
    DistributionWithMean<Vector> tmpInitialBelief,
    AbstractRoadTrackingFilter tmpTrackingFilter);

  public Collection<InferenceGraphEdge> getNearbyEdges(
    Vector projLocation, double radius);

  public InferenceGraphEdge getNullInferredEdge();

  public Path getNullPath();

  public PathEdge getNullPathEdge();

  public Collection<InferenceGraphEdge> getIncomingTransferableEdges(
    InferenceGraphEdge infEdge);

  public Collection<InferenceGraphEdge> getOutgoingTransferableEdges(
    InferenceGraphEdge infEdge);

  public Envelope getProjGraphExtent();

  public PathEdge getPathEdge(InferenceGraphEdge edge, double d,
    Boolean b);

  public Path getInferredPath(PathEdge pathEdge);

  public Path getInferredPath(List<PathEdge> currentPath, Boolean b);

  public boolean edgeHasReverse(Geometry edge);

  public InferenceGraphEdge getInferredEdge(String id);

  public VehicleState createVehicleState(GpsObservation obs,
    AbstractRoadTrackingFilter trackingFilter,
    PathStateDistribution pathStateDistribution,
    OnOffEdgeTransDistribution edgeTransDist, VehicleState parent);

}