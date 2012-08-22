package inference;

import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.math.matrix.mtj.DenseVector;

import java.util.Collections;
import java.util.List;

import org.codehaus.jackson.annotate.JsonIgnore;
import org.codehaus.jackson.map.annotate.JsonSerialize;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.statistics.filters.FilterInformation;
import org.openplans.tools.tracking.impl.statistics.filters.VehicleTrackingFilter;

import api.OsmSegment;

import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;

public class ResultSet {

  static public class EvaluatedPathInfo {
    final List<OsmSegment> pathEdges;
    final double totalDistance;
    final private Integer startEdge;
    final private Integer endEdge;

    public EvaluatedPathInfo(List<OsmSegment> pathEdges,
      double totalDistance, Integer startEdge, Integer endEdge) {
      this.pathEdges = pathEdges;
      this.totalDistance = totalDistance;
      this.startEdge = startEdge;
      this.endEdge = endEdge;
    }

    @JsonSerialize
    public Integer getEndEdge() {
      return endEdge;
    }

    @JsonSerialize
    public List<OsmSegment> getPathEdges() {
      return pathEdges;
    }

    @JsonSerialize
    public Integer getStartEdge() {
      return startEdge;
    }

    @JsonSerialize
    public double getTotalDistance() {
      return totalDistance;
    }

  }

  static public class InferenceResultSet extends ResultSet {

    private final int particleCount;

    public InferenceResultSet(ResultSet infResults, int count) {
      super(infResults.state, infResults.filter,
          infResults.meanCoords, infResults.majorAxisCoords,
          infResults.minorAxisCoords, infResults.pathSegments,
          infResults.pathDirection);
      this.particleCount = count;
    }

    @JsonSerialize
    public int getParticleCount() {
      return this.particleCount;
    }

  }

  private final Coordinate meanCoords;
  private final Coordinate majorAxisCoords;
  private final Coordinate minorAxisCoords;
  private final List<OsmSegmentWithVelocity> pathSegments;

  protected final VehicleTrackingFilter filter;
  protected final VehicleState state;
  private final Double pathDirection;
  private final OsmSegmentWithVelocity inferredEdge;

  //    private final List<EvaluatedPathInfo> evaluatedPaths;

  public ResultSet(VehicleState vehicleState,
    VehicleTrackingFilter filter, Coordinate meanCoords,
    Coordinate majorAxisCoords, Coordinate minorAxisCoords,
    List<OsmSegmentWithVelocity> pathSegments, Double pathDirection) {
    this.meanCoords = meanCoords;
    this.majorAxisCoords = majorAxisCoords;
    this.minorAxisCoords = minorAxisCoords;
    this.pathSegments = pathSegments;
    this.state = vehicleState;
    this.pathDirection = pathDirection;
    this.inferredEdge = createInferredEdge();
    //      this.evaluatedPaths = createEvaluatedPaths();
    this.filter = filter;
  }

  @JsonIgnore
  public List<ResultSet.EvaluatedPathInfo> createEvaluatedPaths() {
    List<ResultSet.EvaluatedPathInfo> pathEdgeIds;
    final FilterInformation filterInfo =
        this.filter.getFilterInformation(this.state.getObservation());
    if (filterInfo != null) {
      pathEdgeIds = Lists.newArrayList();
      for (final InferredPath pathEntry : filterInfo
          .getEvaluatedPaths()) {
        final List<OsmSegment> edges = Lists.newArrayList();
        for (final PathEdge edge : pathEntry.getEdges()) {
          if (edge.getInferredEdge().getEdgeId() != null) {
            final OsmSegment segment =
                new OsmSegment(edge.getInferredEdge().getEdgeId(),
                    edge.getInferredEdge().getGeometry(), edge
                        .getInferredEdge().getEdge().getName());
            edges.add(segment);
          }
        }
        if (!edges.isEmpty()) {
          final Integer startEdge =
              pathEntry.getStartSearchEdge() != null ? pathEntry
                  .getStartSearchEdge().getEdgeId() : null;
          final Integer endEdge =
              pathEntry.getEndSearchEdge() != null ? pathEntry.getEndSearchEdge()
                  .getEdgeId() : null;
          pathEdgeIds.add(new EvaluatedPathInfo(edges, pathEntry
              .getTotalPathDistance(), startEdge, endEdge));
        }
      }
    } else {
      pathEdgeIds = Collections.emptyList();
    }
    return pathEdgeIds;
  }

  @JsonIgnore
  private OsmSegmentWithVelocity createInferredEdge() {
    final OsmSegmentWithVelocity osmSegment;
    final InferredEdge edge = state.getInferredEdge();
    if (edge != InferredEdge.getEmptyEdge()) {
      osmSegment =
          new OsmSegmentWithVelocity(edge.getEdgeId(),
              edge.getGeometry(), edge.getEdge().getName(), edge
                  .getVelocityPrecisionDist().getLocation());
    } else {
      osmSegment =
          new OsmSegmentWithVelocity(-1, null, "empty", null);
    }
    return osmSegment;
  }

  @JsonIgnore
  public VehicleTrackingFilter getFilter() {
    return filter;
  }

  @JsonSerialize
  public OsmSegmentWithVelocity getInferredEdge() {
    return inferredEdge;
  }

  @JsonSerialize
  public Coordinate getMajorAxisCoords() {
    return majorAxisCoords;
  }

  @JsonSerialize
  public Coordinate getMeanCoords() {
    return meanCoords;
  }

  @JsonSerialize
  public Coordinate getMinorAxisCoords() {
    return minorAxisCoords;
  }

  @JsonSerialize
  public Double getPathDirection() {
    return pathDirection;
  }

  @JsonSerialize
  public List<OsmSegmentWithVelocity> getPathSegments() {
    return pathSegments;
  }

  @JsonIgnore
  public VehicleState getState() {
    return state;
  }

  @JsonSerialize
  public double[] getStateCovariance() {
    return ((DenseMatrix) state.getBelief().getCovariance())
        .convertToVector().getArray().clone();
  }

  //    @JsonSerialize
  //    public List<EvaluatedPathInfo> getEvaluatedPaths() {
  //      return evaluatedPaths;
  //    }

  @JsonSerialize
  public double[] getStateMean() {
    return ((DenseVector) state.getBelief().getMean()).getArray()
        .clone();
  }

}