package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.VectorUtil;
import gov.sandia.cognition.math.matrix.mtj.DenseVector;
import gov.sandia.cognition.statistics.bayesian.conjugate.UnivariateGaussianMeanVarianceBayesianEstimator;
import gov.sandia.cognition.statistics.distribution.NormalInverseGammaDistribution;

import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ExecutionException;

import org.geotools.geometry.GeometryBuilder;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;
import org.openplans.tools.tracking.impl.util.GeoUtils;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.routing.core.TraverseMode;
import org.opentripplanner.routing.edgetype.StreetEdge;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.graph.Vertex;
import org.opentripplanner.routing.location.StreetLocation;

import com.google.common.base.Objects;
import com.google.common.base.Preconditions;
import com.google.common.cache.CacheBuilder;
import com.google.common.cache.CacheLoader;
import com.google.common.cache.LoadingCache;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableList.Builder;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.google.common.collect.Sets;
import com.vividsolutions.jts.algorithm.Angle;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateSequence;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.linearref.LengthIndexedLine;
import com.vividsolutions.jts.linearref.LengthLocationMap;
import com.vividsolutions.jts.linearref.LinearLocation;
import com.vividsolutions.jts.linearref.LocationIndexedLine;
import com.vividsolutions.jts.math.Vector2D;
import com.vividsolutions.jts.shape.GeometricShapeBuilder;
import com.vividsolutions.jts.util.GeometricShapeFactory;

public class InferredGraph {

  private final Map<Edge, InferredEdge> edgeToInfo = Maps.newConcurrentMap();
  
  private final Graph graph;

  private final OtpGraph narratedGraph;
  
  public InferredGraph(OtpGraph graph) {
   this.graph = graph.getGraph(); 
   this.narratedGraph = graph;
  }
    
  private static class PathKey {
    
    private final Coordinate startCoord;
    private final Coordinate endCoord;
    
    public PathKey(Coordinate startCoord, Coordinate endCoord) {
      this.startCoord = startCoord;
      this.endCoord = endCoord;
    }

    public Coordinate getStartCoord() {
      return startCoord;
    }

    public Coordinate getEndCoord() {
      return endCoord;
    }

    @Override
    public int hashCode() {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((endCoord == null) ? 0 : endCoord.hashCode());
      result = prime * result
          + ((startCoord == null) ? 0 : startCoord.hashCode());
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
      if (getClass() != obj.getClass()) {
        return false;
      }
      PathKey other = (PathKey) obj;
      if (endCoord == null) {
        if (other.endCoord != null) {
          return false;
        }
      } else if (!endCoord.equals(other.endCoord)) {
        return false;
      }
      if (startCoord == null) {
        if (other.startCoord != null) {
          return false;
        }
      } else if (!startCoord.equals(other.startCoord)) {
        return false;
      }
      return true;
    }
    
  }
  
  private final LoadingCache<PathKey, Set<InferredPath>> pathsCache = CacheBuilder.newBuilder()
      .maximumSize(1000)
      .build(
          new CacheLoader<PathKey, Set<InferredPath>>() {
            public Set<InferredPath> load(PathKey key) {
//              return Sets.newHashSet(InferredPath.getEmptyPath());
              return computePaths(key);
            }
          }); 

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
    
    final Coordinate fromCoord;
    if (fromState.getInferredEdge() == InferredGraph.getEmptyEdge()) {
      fromCoord = GeoUtils.convertToLatLon(fromState.getMeanLocation());
    } else {
      fromCoord = fromState.getInferredEdge().getCenterPointCoord();
    }
    
    PathKey key = new PathKey(fromCoord, toCoord);
    
    return pathsCache.getUnchecked(key);
  }
  
  private Set<InferredPath> computePaths(PathKey key) {
    
    Coordinate fromCoord = GeoUtils.reverseCoordinates(key.getStartCoord());
    Coordinate toCoord = GeoUtils.reverseCoordinates(key.getEndCoord());
    
    Set<InferredPath> paths = Sets.newHashSet(InferredPath.getEmptyPath());
    Builder<PathEdge> path = ImmutableList.builder();
    final CoordinateSequence movementSeq = JTSFactoryFinder
        .getGeometryFactory().getCoordinateSequenceFactory()
        .create(new Coordinate[] { fromCoord, toCoord});
    
    final Geometry movementGeometry = JTSFactoryFinder
        .getGeometryFactory().createLineString(movementSeq);
    
    final List<Edge> minimumConnectingEdges = Objects.firstNonNull(narratedGraph.getStreetMatcher()
        .match(movementGeometry), ImmutableList.<Edge> of());
    
    if (!minimumConnectingEdges.isEmpty()) {
      double pathDist = 0d;
      for (Edge pathEdge : minimumConnectingEdges) {
        path.add(PathEdge.getEdge(this.getInferredEdge(pathEdge), pathDist));
        pathDist += pathEdge.getDistance();
      }
      paths.add(new InferredPath(path.build(), pathDist));
    }
    return paths;
  }
  
  /**
   * Get nearby street edges from a projected point.
   * @param mean
   * @return
   */
  public Set<InferredEdge> getNearbyEdges(Vector mean) {
    Set<InferredEdge> results = Sets.newHashSet();
    Coordinate latlon = GeoUtils.convertToLatLon(mean);
    List<StreetEdge> snappedEdges = this.narratedGraph.snapToGraph(null, latlon);
    for (Edge edge : snappedEdges) {
      results.add(getInferredEdge(edge));
    }
    return results;
  }
  
  public InferredEdge getInferredEdge(Edge edge) {

    InferredEdge edgeInfo = edgeToInfo.get(edge);
    final Integer edgeId = graph.getIdForEdge(edge);

    if (edgeInfo == null) {
      edgeInfo = new InferredEdge(edge, edgeId, this);
      edgeToInfo.put(edge, edgeInfo);
    }

    return edgeInfo;
  }

  public InferredEdge getEdge(int id) {

    final Edge edge = graph.getEdgeById(id);
    InferredEdge edgeInfo = edgeToInfo.get(edge);

    if (edgeInfo == null) {
      edgeInfo = new InferredEdge(edge, id, this);
      edgeToInfo.put(edge, edgeInfo);
    }

    return edgeInfo;
  }
  
  public Graph getGraph() {
    return graph;
  }
  
  
  public static class InferredEdge {
  
    private final LocationIndexedLine locationIndexedLine;
    private final Integer edgeId;
    private final Vertex startVertex;
    private final Vertex endVertex;
    private final Vector endPoint;
    private final Vector startPoint;
    private final double length;
    private final NormalInverseGammaDistribution velocityPrecisionDist;
    private final UnivariateGaussianMeanVarianceBayesianEstimator velocityEstimator;
    private final Geometry geometry;
    private final InferredGraph graph;
    private final LengthIndexedLine lengthIndexedLine;
    private final LengthLocationMap lengthLocationMap;
    
    /*
     * This is the empty edge, which stands for free movement
     */
    private final static InferredGraph.InferredEdge emptyEdge = new InferredGraph.InferredEdge();
  
  
    private InferredEdge() {
      this.edgeId = null;
      this.locationIndexedLine = null;
      this.endPoint = null;
      this.startPoint = null;
      this.length = 0;
      this.velocityEstimator = null;
      this.velocityPrecisionDist = null;
      this.startVertex = null;
      this.endVertex = null;
      this.geometry = null;
      this.graph = null;
      this.lengthIndexedLine = null;
      this.lengthLocationMap = null;
    }
  
    private InferredEdge(Edge edge, Integer edgeId, InferredGraph graph) {
      this.graph = graph;
      this.edgeId = edgeId;
      
      /*
       * Warning: this geometry is in lon/lat and may contain more than
       * one straight line.
       */
      this.geometry = edge.getGeometry();
      
      this.locationIndexedLine = new LocationIndexedLine(edge.getGeometry());
      this.lengthIndexedLine = new LengthIndexedLine(edge.getGeometry());
      this.lengthLocationMap = new LengthLocationMap(geometry);
      
      this.startVertex = edge.getFromVertex();
      this.endVertex = edge.getToVertex();
      
      final Coordinate startPoint = this.locationIndexedLine.extractPoint(this.locationIndexedLine
          .getStartIndex());
      /*
       * We need to flip these coords around to get lat/lon.
       */
      final Coordinate startPointCoord = GeoUtils.convertToEuclidean(
          new Coordinate(startPoint.y, startPoint.x));
      this.startPoint  = VectorFactory.getDefault().createVector2D(startPointCoord.x,
          startPointCoord.y);
      
      final Coordinate endPoint = this.locationIndexedLine.extractPoint(this.locationIndexedLine.getEndIndex());
      final Coordinate endPointCoord = GeoUtils.convertToEuclidean(
          new Coordinate(endPoint.y, endPoint.x));
      this.endPoint  = VectorFactory.getDefault().createVector2D(endPointCoord.x,
          endPointCoord.y);
      
      this.length = GeoUtils.getAngleDegreesInMeters(geometry.getLength());
      
      this.velocityPrecisionDist =
        // ~4.4 m/s, std. dev ~ 30 m/s, Gamma with exp. value = 30 m/s
        // TODO perhaps variance of velocity should be in m/s^2. yeah...
        new NormalInverseGammaDistribution(266d, 1 / Math.sqrt(1800d),
            1 / Math.sqrt(1800) + 1, Math.sqrt(1800));
      this.velocityEstimator = new UnivariateGaussianMeanVarianceBayesianEstimator(
          velocityPrecisionDist);
    }
  
    public Integer getEdgeId() {
      return edgeId;
    }
  
    /**
     * Get the snapped location in projected/euclidean coordinates for the 
     * given obsPoint (in lat/lon).
     * @param obsPoint
     * @return
     */
    public Vector getPointOnEdge(Coordinate obsPoint) {
      if (this == InferredEdge.emptyEdge)
        return null;
      final Coordinate revObsPoint = new Coordinate(obsPoint.y, obsPoint.x);
      final LinearLocation here = locationIndexedLine.project(revObsPoint);
      final Coordinate pointOnLine = locationIndexedLine.extractPoint(here);
      final Coordinate revOnLine = new Coordinate(pointOnLine.y, pointOnLine.x);
      final Coordinate projPointOnLine = GeoUtils.convertToEuclidean(revOnLine);
      return VectorFactory.getDefault().createVector2D(projPointOnLine.x,
          projPointOnLine.y);
    }
  
    /**
     * This returns a list of edges that are incoming, wrt the direction of this edge,
     * and that are reachable from this edge (e.g. not one way in the direction of this edge).
     * @return
     */
    public List<InferredEdge> getIncomingTransferableEdges() {
      
      List<InferredEdge> result = Lists.newArrayList();
      for (Edge edge : this.startVertex.getIncoming()) {
        
        Set<Edge> tmpResults = Sets.newHashSet();
        if (edge.getGeometry() == null
            || !edge.getMode().equals(TraverseMode.CAR)) {
          /*
           * Only attempt one level of descent for finding street edges
           */
          tmpResults.addAll(edge.getFromVertex().getOutgoingStreetEdges());
//          tmpResults.addAll(edge.getToVertex().getOutgoingStreetEdges());
        } else {
          tmpResults.add(edge);
        }
        for (Edge edge2 : tmpResults) {
          if (!edge2.getGeometry().equals(geometry))
            result.add(graph.getInferredEdge(edge2));
        }
      }
      
      return result;
    }
    
    /**
     * This returns a list of edges that are outgoing, wrt the direction of this edge,
     * and that are reachable from this edge (e.g. not one way against the direction of this edge).
     * @return
     */
    public List<InferredEdge> getOutgoingTransferableEdges() {
      List<InferredEdge> result = Lists.newArrayList();
      for (Edge edge : this.endVertex.getOutgoingStreetEdges()) {
        
//        Set<Edge> tmpResults = Sets.newHashSet();
//        if (edge.getGeometry() == null
//            || !edge.getMode().equals(TraverseMode.CAR)) {
//          /*
//           * Only attempt one level of descent for finding street edges
//           */
//          tmpResults.addAll(edge.getFromVertex().getOutgoingStreetEdges());
//          tmpResults.addAll(edge.getToVertex().getOutgoingStreetEdges());
//        } else {
//          tmpResults.add(edge);
//        }
//        for (Edge edge2 : tmpResults) {
//          if (!edge2.getGeometry().equals(geometry))
            result.add(graph.getInferredEdge(edge));
//        }
      }
      
      return result;
    }
    
    public Vector getEndPoint() {
      return this.endPoint;
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

    public Vertex getStartVertex() {
      return startVertex;
    }

    public Vertex getEndVertex() {
      return endVertex;
    }

    public Coordinate getCenterPointCoord() {
      return this.geometry.getCentroid().getCoordinate();
    }

    public Geometry getGeometry() {
      return geometry;
    }

    @Override
    public int hashCode() {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((geometry == null) ? 0 : geometry.hashCode());
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
      if (getClass() != obj.getClass()) {
        return false;
      }
      InferredEdge other = (InferredEdge) obj;
      if (geometry == null) {
        if (other.geometry != null) {
          return false;
        }
      } else if (!geometry.equals(other.geometry)) {
        return false;
      }
      return true;
    }

    @Override
    public String toString() {
      return "InferredEdge [edgeId=" + edgeId + ", endPoint=" + endPoint
          + ", startPoint=" + startPoint + ", length=" + length + "]";
    }

    public LocationIndexedLine getLocationIndexedLine() {
      return locationIndexedLine;
    }

    public InferredGraph getGraph() {
      return graph;
    }

    public LengthIndexedLine getLengthIndexedLine() {
      return lengthIndexedLine;
    }

    public LengthLocationMap getLengthLocationMap() {
      return lengthLocationMap;
    }

    public static InferredGraph.InferredEdge getEmptyedge() {
      return emptyEdge;
    }

  
  }

  public static InferredEdge getEmptyEdge() {
    return InferredEdge.emptyEdge;
  }

  public OtpGraph getNarratedGraph() {
    return narratedGraph;
  }



}
