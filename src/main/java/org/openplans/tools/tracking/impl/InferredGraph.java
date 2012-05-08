package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.statistics.bayesian.conjugate.UnivariateGaussianMeanVarianceBayesianEstimator;
import gov.sandia.cognition.statistics.distribution.NormalInverseGammaDistribution;

import java.util.List;
import java.util.Map;

import org.geotools.geometry.jts.JTSFactoryFinder;
import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;
import org.openplans.tools.tracking.impl.util.GeoUtils;
import org.openplans.tools.tracking.impl.util.OtpGraph;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;

import com.google.common.base.Objects;
import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableList.Builder;
import com.google.common.collect.Iterables;
import com.google.common.collect.Maps;
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
   * 
   * @param fromState
   * @param toCoord
   * @param snappedEdges
   * @return
   */
  public ImmutableList<InferredEdge> getPath(VehicleState fromState, Coordinate toCoord) {
    Preconditions.checkNotNull(fromState);
    Builder<InferredEdge> path = ImmutableList.builder();
    
    
    // TODO FIXME use some kind of caching
    Coordinate fromCoord = GeoUtils.convertToLatLon(fromState.getMeanLocation());
    final CoordinateSequence movementSeq = JTSFactoryFinder
        .getGeometryFactory().getCoordinateSequenceFactory()
        .create(new Coordinate[] { fromCoord, toCoord });
    final Geometry movementGeometry = JTSFactoryFinder
        .getGeometryFactory().createLineString(movementSeq);
    
    // TODO does this work when the source isn't on an edge?
    final List<Edge> minimumConnectingEdges = Objects.firstNonNull(narratedGraph.getStreetMatcher()
        .match(movementGeometry), ImmutableList.<Edge> of());
    
    // TODO does the above path contain the source edge?
//    path.add(Iterables.getLast(fromState.getInferredPath()));
    
    if (minimumConnectingEdges.isEmpty()) {
      path.add(InferredGraph.emptyEdge);
    } else {
      for (Edge edge : minimumConnectingEdges) {
        path.add(this.getEdge(edge));
      }
    }
    
    return path.build();
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
    private final NormalInverseGammaDistribution velocityPrecisionDist;
  
    private final UnivariateGaussianMeanVarianceBayesianEstimator velocityEstimator;
  
  
    private final Edge edge;
    private final LocationIndexedLine line;
    private final int edgeId;
  
    /*
     * Angle between this line and the Y-axis
     */
    private final Double angle;
  
  
    private InferredEdge() {
      this.edge = null;
      this.edgeId = -1;
      this.angle = null;
      this.line = null;
      this.velocityEstimator = null;
      this.velocityPrecisionDist = null;
    }
  
    private InferredEdge(Edge edge, int edgeId) {
      this.edge = edge;
      this.edgeId = edgeId;
      this.line = new LocationIndexedLine(this.edge.getGeometry());
      final Coordinate startPoint = this.line.extractPoint(this.line
          .getStartIndex());
      final Coordinate endPoint = this.line.extractPoint(this.line.getEndIndex());
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
  
    public Coordinate getPointOnEdge(Coordinate obsPoint) {
      if (this == emptyEdge)
        return null;
      final LinearLocation here = line.project(obsPoint);
      final Coordinate pointOnLine = line.extractPoint(here);
      return pointOnLine;
    }
  
    public UnivariateGaussianMeanVarianceBayesianEstimator getVelocityEstimator() {
      return velocityEstimator;
    }
  
    public NormalInverseGammaDistribution getVelocityPrecisionDist() {
      if (this == emptyEdge)
        return null;
      return velocityPrecisionDist;
    }
  
    public void updateVelocity(double varianceDist) {
      if (this == emptyEdge)
        return;
//      Preconditions.checkArgument(this != emptyEdge);
      /*
       * TODO should have a gamma for "observed" variance
       */
      velocityEstimator.update(velocityPrecisionDist, varianceDist);
    }

    @Override
    public String toString() {
      return "InferredEdge [velocityPrecisionDist=" + velocityPrecisionDist
          + ", velocityEstimator=" + velocityEstimator + ", edge=" + edge
          + ", line=" + line + ", edgeId=" + edgeId + ", angle=" + angle + "]";
    }
  
  }

  public static InferredEdge getEmptyEdge() {
    return emptyEdge;
  }

  public OtpGraph getNarratedGraph() {
    return narratedGraph;
  }


}
