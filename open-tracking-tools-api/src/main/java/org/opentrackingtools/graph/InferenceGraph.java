package org.opentrackingtools.graph;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.DistributionWithMean;

import java.util.Collection;
import java.util.List;

import org.geotools.graph.build.line.DirectedLineStringGraphGenerator;
import org.geotools.graph.structure.Edge;
import org.geotools.graph.structure.Node;
import org.geotools.graph.structure.basic.BasicDirectedNode;
import org.geotools.graph.traverse.standard.AStarIterator.AStarFunctions;
import org.geotools.graph.traverse.standard.AStarIterator.AStarNode;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.paths.Path;

import com.google.common.primitives.Doubles;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Point;

public interface InferenceGraph {

  public static class VehicleStateAStarFunction extends
      AStarFunctions {
  
    final double distanceToTravel;
    final double obsStdDevDistance;
    final Coordinate toCoord;
  
    public VehicleStateAStarFunction(Node destination,
      Coordinate toCoord, double obsStdDevDistance,
      double distanceToTravel) {
      super(destination);
      this.toCoord = toCoord;
      this.distanceToTravel = distanceToTravel;
      this.obsStdDevDistance = obsStdDevDistance;
    }
  
    @Override
    public double cost(AStarNode n1, AStarNode n2) {
      final BasicDirectedNode dn1 = (BasicDirectedNode) n1.getNode();
      final BasicDirectedNode dn2 = (BasicDirectedNode) n2.getNode();
  
      /*
       * TODO are we handling multiple edges correctly?
       */
      final List<Edge> edgesBetween = dn1.getOutEdges(dn2);
  
      /*
       * Make sure this direction is traversable
       */
      if (edgesBetween.isEmpty()) {
        return Double.POSITIVE_INFINITY;
      }
  
      /*
       * TODO
       * Compute distance past projected value?
       */
      final double[] lengths = new double[edgesBetween.size()];
      for (int i = 0; i < lengths.length; i++) {
        lengths[i] =
            ((LineString) edgesBetween.get(i).getObject())
                .getLength();
      }
      final double totalDistanceTraveled =
          Doubles.min(lengths) + n1.getG();
  
      final double cost =
          Math.abs(Math.min(0, this.distanceToTravel
              - totalDistanceTraveled));
      return cost;
    }
  
    @Override
    public double h(Node n) {
  
      final double distance =
          ((Point) n.getObject()).getCoordinate().distance(
              this.toCoord);
  
      if (distance < this.obsStdDevDistance) {
        return 0d;
      }
  
      return (distance - this.obsStdDevDistance) / 15d; // 15 m/s, ~35 mph, a random driving speed
    }
  }

  /**
   * Assuming that the LineString is mostly constant allows us to cache values
   * like getLength, which otherwise, over time, build up needless calculations.
   * If the internal values happen to change, then we update the cached values
   * anyway.
   * 
   * @author bwillard
   * 
   */
  static public class ConstLineString extends LineString {
  
    private static final long serialVersionUID = 1114083576711858849L;
  
    double length;
  
    public ConstLineString(LineString projectedEdge) {
      super(projectedEdge.getCoordinateSequence(), projectedEdge
          .getFactory());
      this.length = projectedEdge.getLength();
    }
  
    @Override
    protected void geometryChangedAction() {
      super.geometryChangedAction();
      this.length = super.getLength();
    }
  
    @Override
    public double getLength() {
      return this.length;
    }
  }

  static public class StrictLineStringGraphGenerator extends
      DirectedLineStringGraphGenerator {
  
    public StrictLineStringGraphGenerator() {
      super();
      this.setGraphBuilder(new StrictDirectedGraphBuilder());
    }
  }

  public boolean edgeHasReverse(Geometry edge);

  public Envelope getGPSGraphExtent();

  public Collection<InferenceGraphEdge> getIncomingTransferableEdges(
    InferenceGraphEdge infEdge);

  public InferenceGraphEdge getInferenceGraphEdge(String id);

  public Collection<InferenceGraphSegment> getNearbyEdges(
    Coordinate projLocation, double radius);

  public Collection<InferenceGraphSegment> getNearbyEdges(
    DistributionWithMean<Vector> tmpInitialBelief, Matrix covariance);

  public Collection<InferenceGraphSegment> getNearbyEdges(
    Vector projLocation, double radius);

  public Collection<InferenceGraphEdge> getOutgoingTransferableEdges(
    InferenceGraphEdge infEdge);

  public Collection<Path> getPaths(
    VehicleStateDistribution<? extends GpsObservation> fromState,
    GpsObservation toCoord);

  public Envelope getProjGraphExtent();

  public Collection<InferenceGraphEdge> getTopoEquivEdges(
    InferenceGraphEdge edge);

}