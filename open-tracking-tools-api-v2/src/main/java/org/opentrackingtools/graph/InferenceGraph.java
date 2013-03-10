package org.opentrackingtools.graph;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.DistributionWithMean;

import java.util.Collection;

import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.Path;

import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;

public interface InferenceGraph {

  public Collection<Path> getPaths(
    VehicleState<? extends GpsObservation> fromState,
    GpsObservation toCoord);

  public Collection<InferenceGraphEdge> getTopoEquivEdges(
    InferenceGraphEdge edge);

  public Envelope getGPSGraphExtent();

  public Collection<InferenceGraphEdge> getNearbyEdges(
    DistributionWithMean<Vector> tmpInitialBelief, Matrix covariance);

  public Collection<InferenceGraphEdge> getNearbyEdges(
    Vector projLocation, double radius);

  public Collection<InferenceGraphEdge> getIncomingTransferableEdges(
    InferenceGraphEdge infEdge);

  public Collection<InferenceGraphEdge> getOutgoingTransferableEdges(
    InferenceGraphEdge infEdge);

  public Envelope getProjGraphExtent();

  public boolean edgeHasReverse(Geometry edge);

}