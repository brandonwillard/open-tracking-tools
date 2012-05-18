package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.VectorUtil;
import gov.sandia.cognition.statistics.bayesian.conjugate.UnivariateGaussianMeanVarianceBayesianEstimator;
import gov.sandia.cognition.statistics.distribution.NormalInverseGammaDistribution;

import java.util.List;
import java.util.Map;
import java.util.Set;

import org.geotools.geometry.jts.JTSFactoryFinder;
import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;
import org.openplans.tools.tracking.impl.util.GeoUtils;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.Vertex;
import org.opentripplanner.routing.location.StreetLocation;

import com.google.common.base.Objects;
import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableList.Builder;
import com.google.common.collect.Iterables;
import com.google.common.collect.Maps;
import com.google.common.collect.Sets;
import com.vividsolutions.jts.algorithm.Angle;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateSequence;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.linearref.LinearLocation;
import com.vividsolutions.jts.linearref.LocationIndexedLine;

public class InferredGraph {

  private final Map<Edge, InferredEdge> edgeToInfo = Maps.newConcurrentMap();
  
  private final Graph graph;

  private final OtpGraph narratedGraph;
  
  public InferredGraph(OtpGraph graph) {
   // TODO FIXME need to make interfaces and get rid of two graphs.
   this.graph = graph.getGraph(); 
   this.narratedGraph = graph;
  }
    
  /*
   * This is the empty edge, which stands for free movement
   */
  private final static InferredEdge emptyEdge = new InferredEdge();

  /**
   * Compute a path of InferredEdge's between observations.  
   * This includes the empty edge.
   * Also, it finds the edges nearest to the toCoord and gets paths
   * to the center points of those edges.
   * 
   * @param fromState
   * @param toCoord
   * @return
   */
  public Set<InferredPath> getPaths(VehicleState fromState, Coordinate toCoord) {
    Preconditions.checkNotNull(fromState);
    
    
    // TODO use some kind of caching
    Coordinate fromCoord = GeoUtils.convertToLatLon(
        VectorFactory.getDefault().createVector2D(fromState.getMeanLocation().getElement(0),
            fromState.getMeanLocation().getElement(1)));
    final Vertex snappedVertex = this.narratedGraph.getIndexService().getClosestVertex(toCoord,
        null, this.narratedGraph.getOptions());
    
    Set<InferredPath> paths = Sets.newHashSet(InferredPath.getEmptyPath());
    if (snappedVertex != null && (snappedVertex instanceof StreetLocation)) {
      Builder<PathEdge> path = ImmutableList.builder();
      final StreetLocation snappedStreetLocation = (StreetLocation) snappedVertex;
      
      for (final Edge edge : Objects.firstNonNull(
          snappedStreetLocation.getOutgoingStreetEdges(),
          ImmutableList.<Edge> of())) {
        Coordinate edgeCenter = edge.getGeometry().getCentroid().getCoordinate();
        final CoordinateSequence movementSeq = JTSFactoryFinder
            .getGeometryFactory().getCoordinateSequenceFactory()
            .create(new Coordinate[] { fromCoord, edgeCenter});
        
        final Geometry movementGeometry = JTSFactoryFinder
            .getGeometryFactory().createLineString(movementSeq);
        
        final List<Edge> minimumConnectingEdges = Objects.firstNonNull(narratedGraph.getStreetMatcher()
            .match(movementGeometry), ImmutableList.<Edge> of());
        
        Double pathDist;
        if (minimumConnectingEdges.isEmpty()) {
          pathDist = null;
          path.add(PathEdge.getEmptyPathEdge());
        } else {
          pathDist = 0d;
          for (Edge pathEdge : minimumConnectingEdges) {
            path.add(PathEdge.getEdge(this.getEdge(pathEdge), pathDist));
            pathDist += pathEdge.getDistance();
          }
        }
        paths.add(new InferredPath(path.build(), pathDist));
      }
    }
    return paths;
  }
  
  public InferredEdge getEdge(Edge edge) {

    InferredEdge edgeInfo = edgeToInfo.get(edge);
    final int edgeId = graph.getIdForEdge(edge);

    if (edgeInfo == null) {
      edgeInfo = new InferredEdge(edge, edgeId);
      edgeToInfo.put(edge, edgeInfo);
    }

    return edgeInfo;
  }

  public InferredEdge getEdge(int id) {

    final Edge edge = graph.getEdgeById(id);
    InferredEdge edgeInfo = edgeToInfo.get(edge);

    if (edgeInfo == null) {
      edgeInfo = new InferredEdge(edge, id);
      edgeToInfo.put(edge, edgeInfo);
    }

    return edgeInfo;
  }
  
  public Graph getGraph() {
    return graph;
  }
  
  
  public static class InferredEdge {
  
    private final Edge edge;
    private final LocationIndexedLine line;
    private final int edgeId;
  
    /*
     * Angle between this line and the Y-axis
     */
    private final Double angle;

    private final Vector endPoint;
    private final Vector startPoint;
    private final double length;
    private final NormalInverseGammaDistribution velocityPrecisionDist;
    private final UnivariateGaussianMeanVarianceBayesianEstimator velocityEstimator;
  
  
    private InferredEdge() {
      this.edge = null;
      this.edgeId = -1;
      this.angle = null;
      this.line = null;
      this.endPoint = null;
      this.startPoint = null;
      this.length = 0;
      this.velocityEstimator = null;
      this.velocityPrecisionDist = null;
    }
  
    private InferredEdge(Edge edge, int edgeId) {
      this.edge = edge;
      this.edgeId = edgeId;
      this.line = new LocationIndexedLine(this.edge.getGeometry());
      
      final Coordinate startPoint = this.line.extractPoint(this.line
          .getStartIndex());
      final Coordinate startPointCoord = GeoUtils.convertToEuclidean(startPoint);
      this.startPoint  = VectorFactory.getDefault().createVector2D(startPointCoord.x,
          startPointCoord.y);
      
      final Coordinate endPoint = this.line.extractPoint(this.line.getEndIndex());
      final Coordinate endPointCoord = GeoUtils.convertToEuclidean(endPoint);
      this.endPoint  = VectorFactory.getDefault().createVector2D(endPointCoord.x,
          endPointCoord.y);
      
      this.length = this.startPoint.euclideanDistance(this.endPoint);
      
      this.angle = Angle.angle(startPoint, endPoint);
      
      this.velocityPrecisionDist =
        // ~4.4 m/s, std. dev ~ 30 m/s, Gamma with exp. value = 30 m/s
        // TODO perhaps variance of velocity should be in m/s^2. yeah...
        new NormalInverseGammaDistribution(266d, 1 / Math.sqrt(1800d),
            1 / Math.sqrt(1800) + 1, Math.sqrt(1800));
      this.velocityEstimator = new UnivariateGaussianMeanVarianceBayesianEstimator(
          velocityPrecisionDist);
    }
  
    public Double getAngle() {
      return angle;
    }
  
    public Edge getEdge() {
      return edge;
    }
  
    public int getEdgeId() {
      return edgeId;
    }
  
    public LocationIndexedLine getLine() {
      return line;
    }
  
    /**
     * Get the snapped location in projected/euclidean coordinates for the 
     * given obsPoint (in lat/lon).
     * @param obsPoint
     * @return
     */
    public Vector getPointOnEdge(Coordinate obsPoint) {
      if (this == emptyEdge)
        return null;
      final LinearLocation here = line.project(obsPoint);
      final Coordinate pointOnLine = line.extractPoint(here);
      final Coordinate projPointOnLine = GeoUtils.convertToEuclidean(pointOnLine);
      return VectorFactory.getDefault().createVector2D(projPointOnLine.x,
          projPointOnLine.y);
    }
  
    public Vector getEndPoint() {
      return this.endPoint;
    }

    @Override
    public String toString() {
      if (this == emptyEdge)
        return "InferredEdge [empty edge]";
      return "InferredEdge [edge=" + edge + ", line=" + line + ", edgeId="
          + edgeId + ", angle=" + angle + ", endPoint=" + endPoint
          + ", startPoint=" + startPoint + ", length=" + length + "]";
    }

    public Vector getStartPoint() {
      return startPoint;
    }

    public double getLength() {
      return length;
    }

    public NormalInverseGammaDistribution getVelocityPrecisionDist() {
      return velocityPrecisionDist;
    }

    public UnivariateGaussianMeanVarianceBayesianEstimator getVelocityEstimator() {
      return velocityEstimator;
    }
  
  }

  public static InferredEdge getEmptyEdge() {
    return emptyEdge;
  }

  public OtpGraph getNarratedGraph() {
    return narratedGraph;
  }


}
