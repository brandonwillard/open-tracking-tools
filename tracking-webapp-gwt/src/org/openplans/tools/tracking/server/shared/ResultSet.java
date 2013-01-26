package org.openplans.tools.tracking.server.shared;

import gov.sandia.cognition.math.matrix.mtj.AbstractMTJMatrix;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.statistics.distribution.InverseWishartDistribution;

import java.util.Collections;
import java.util.List;
import java.util.Map;

import org.codehaus.jackson.annotate.JsonIgnore;
import org.codehaus.jackson.map.annotate.JsonSerialize;
import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.edges.InferredEdge;
import org.opentrackingtools.graph.edges.impl.SimpleInferredEdge;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.edges.impl.EdgePredictiveResults;
import org.opentrackingtools.graph.paths.states.PathState;
import org.opentrackingtools.graph.paths.states.PathStateBelief;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.statistics.filters.vehicles.VehicleTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.impl.FilterInformation;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.ErrorEstimatingRoadTrackingFilter;

import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.vividsolutions.jts.geom.Coordinate;

public class ResultSet {

  static public class EvaluatedPathInfo {
    final List<OsmSegment> pathEdges;
    final double totalDistance;
    final private Integer startEdge;
    final private Integer endEdge;

    public EvaluatedPathInfo(List<OsmSegment> pathEdges,
      double totalDistance, Integer startEdge,
      Integer endEdge) {
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
    private final List<OffRoadPath> offRoadPaths;

    public InferenceResultSet(ResultSet infResults,
      int count, List<OffRoadPath> offRoadPaths) {
      super(infResults.state, infResults.filter,
          infResults.meanCoords,
          infResults.majorAxisCoords,
          infResults.minorAxisCoords,
          infResults.pathSegments, infResults.pathDirection);
      this.particleCount = count;
      this.offRoadPaths = offRoadPaths;
    }

    @JsonSerialize
    public Map<String, Object> getCurrentStateSample() {
      if (this.state.getMovementFilter() instanceof ErrorEstimatingRoadTrackingFilter) {
        final ErrorEstimatingRoadTrackingFilter eeFilter =
            (ErrorEstimatingRoadTrackingFilter) state
                .getMovementFilter();
        if (eeFilter.getCurrentStateSample() != null) {
          return getJsonForPathState(eeFilter
              .getCurrentStateSample());
        }
      }
      return null;
    }

    @JsonSerialize
    public Map<String, Object> getObsCovarPrior() {
      if (state.getMovementFilter() instanceof ErrorEstimatingRoadTrackingFilter) {
        final ErrorEstimatingRoadTrackingFilter eeFilter =
            (ErrorEstimatingRoadTrackingFilter) state
                .getMovementFilter();
        return getJsonForInvWishart(eeFilter
            .getObsVariancePrior());
      } else {
        return null;
      }
    }

    @JsonSerialize
    public Map<String, Object> getOffRoadCovarPrior() {
      if (this.state.getMovementFilter() instanceof ErrorEstimatingRoadTrackingFilter) {
        final ErrorEstimatingRoadTrackingFilter eeFilter =
            (ErrorEstimatingRoadTrackingFilter) state
                .getMovementFilter();
        return getJsonForInvWishart(eeFilter
            .getOffRoadStateVariancePrior());
      } else {
        return null;
      }
    }

    @JsonSerialize
    public double[] getObsCovariance() {
      return ((DenseMatrix) state.getMovementFilter()
          .getObsCovar()).convertToVector().getArray()
          .clone();
    }
  
    @JsonSerialize
    public double[] getOffRoadStateCovariance() {
      return ((DenseMatrix) state.getMovementFilter()
          .getOffRoadStateTransCovar()).convertToVector()
          .getArray().clone();
    }
  
    @JsonSerialize
    public double[] getOnRoadStateCovariance() {
      return ((DenseMatrix) state.getMovementFilter()
          .getOnRoadStateTransCovar()).convertToVector()
          .getArray().clone();
    }
  
    @JsonSerialize
    public double[] getStateQgCovariance() {
      return ((DenseMatrix) state.getMovementFilter().getQg())
          .convertToVector().getArray().clone();
    }
  
    @JsonSerialize
    public double[] getStateQrCovariance() {
      return ((DenseMatrix) state.getMovementFilter().getQr())
          .convertToVector().getArray().clone();
    }
  
    @JsonSerialize
    public List<OffRoadPath> getOffRoadPaths() {
      return offRoadPaths;
    }

    @JsonSerialize
    public double[] getOffRoadTransProbs() {
      return ((gov.sandia.cognition.math.matrix.mtj.DenseVector) this.state
          .getEdgeTransitionDist()
          .getFreeMotionTransPrior().getParameters())
          .getArray();
    }

    @JsonSerialize
    public double[] getOffRoadTransProbsPriorParams() {
      return ((gov.sandia.cognition.math.matrix.mtj.DenseVector) this.state
          .getEdgeTransitionDist()
          .getFreeMotionTransProbPrior().getParameters())
          .getArray();
    }

    @JsonSerialize
    public Map<String, Object> getOnRoadCovarPrior() {
      if (state.getMovementFilter() instanceof ErrorEstimatingRoadTrackingFilter) {
        final ErrorEstimatingRoadTrackingFilter eeFilter =
            (ErrorEstimatingRoadTrackingFilter) state
                .getMovementFilter();
        return getJsonForInvWishart(eeFilter
            .getOnRoadStateVariancePrior());
      } else {
        return null;
      }
    }

    @JsonSerialize
    public double[] getOnRoadTransProbs() {
      return ((gov.sandia.cognition.math.matrix.mtj.DenseVector) this.state
          .getEdgeTransitionDist()
          .getEdgeMotionTransPrior().getParameters())
          .getArray();
    }

    @JsonSerialize
    public double[] getOnRoadTransProbsPriorParams() {
      return ((gov.sandia.cognition.math.matrix.mtj.DenseVector) this.state
          .getEdgeTransitionDist()
          .getEdgeMotionTransProbPrior().getParameters())
          .getArray();
    }

    @JsonSerialize
    public int getParticleCount() {
      return this.particleCount;
    }

    @JsonSerialize
    public Map<String, Object> getPrevStateSample() {
      if (this.state.getMovementFilter() instanceof ErrorEstimatingRoadTrackingFilter) {
        final ErrorEstimatingRoadTrackingFilter eeFilter =
            (ErrorEstimatingRoadTrackingFilter) state
                .getMovementFilter();
        if (eeFilter.getPrevStateSample() != null) {
          return getJsonForPathState(eeFilter
              .getPrevStateSample());
        }
      }
      return null;
    }
  }

  static public class OffRoadPath {
    private GpsObservation startObs;
    private GpsObservation endObs;
    private OsmSegment startEdge;
    private OsmSegment endEdge;

    private List<Coordinate> pointsBetween;

    public OffRoadPath() {
    }

    public OffRoadPath(OffRoadPath last) {
      this.startEdge = last.startEdge;
      this.endEdge = last.endEdge;
      this.startObs = last.startObs;
      this.endObs = last.endObs;
      this.pointsBetween =
          Lists.newArrayList(last.pointsBetween);
    }

    public OsmSegment getEndEdge() {
      return endEdge;
    }

    public Long getEndObsTime() {
      return endObs != null ? endObs.getTimestamp()
          .getTime() : null;
    }

    public List<Coordinate> getPointsBetween() {
      return pointsBetween;
    }

    public OsmSegment getStartEdge() {
      return startEdge;
    }

    public Long getStartObsTime() {
      return startObs != null ? startObs.getTimestamp()
          .getTime() : null;
    }

    public void setEndEdge(OsmSegment endEdge) {
      this.endEdge = endEdge;
    }

    public void setEndObs(GpsObservation endObs) {
      this.endObs = endObs;
    }

    public void setPointsBetween(
      List<Coordinate> pointsBetween) {
      this.pointsBetween = pointsBetween;
    }

    public void setStartEdge(OsmSegment startEdge) {
      this.startEdge = startEdge;
    }

    public void setStartObs(GpsObservation startObs) {
      this.startObs = startObs;
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
    List<OsmSegmentWithVelocity> pathSegments,
    Double pathDirection) {
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
  public List<ResultSet.EvaluatedPathInfo>
      createEvaluatedPaths() {
    List<ResultSet.EvaluatedPathInfo> pathEdgeIds;
    final FilterInformation filterInfo =
        this.filter.getFilterInformation(this.state
            .getObservation());
    if (filterInfo != null) {
      pathEdgeIds = Lists.newArrayList();
      for (final InferredPath pathEntry : filterInfo
          .getEvaluatedPaths()) {
        final List<OsmSegment> edges = Lists.newArrayList();
        for (final PathEdge edge : pathEntry.getPathEdges()) {
          if (edge.getInferredEdge().getEdgeId() != null) {
            final OsmSegment segment =
                new OsmSegment(edge.getInferredEdge());
            edges.add(segment);
          }
        }
        if (!edges.isEmpty()) {
          pathEdgeIds
              .add(new EvaluatedPathInfo(edges, pathEntry
                  .getTotalPathDistance(), null, null));
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
    final InferredEdge edge =
        state.getBelief().getEdge().getInferredEdge();
    if (edge.isNullEdge()) {
      final double velocity;
      if (edge instanceof SimpleInferredEdge)
        velocity = ((SimpleInferredEdge)edge).getVelocityPrecisionDist().getLocation();
      else
        velocity = Double.NaN;
      osmSegment = new OsmSegmentWithVelocity(edge, velocity);
              
    } else {
      osmSegment =
          new OsmSegmentWithVelocity("-1", null, "empty",
              null);
    }
    return osmSegment;
  }

  @JsonSerialize
  public Map<String, Object> getPriorPredictiveResults() {
    final Map<String, Object> jsonData = Maps.newHashMap();
    
    EdgePredictiveResults result = this.filter.getFilterInformation(state.getObservation()).
        getEdgePredictiveResult(state.getParentState(), state.getBelief().getEdge());
    
    if (result == null)
      return null;
    
    jsonData.put("edgeMarginalLogLik", result.getEdgePredMarginalLogLik());
    jsonData.put("edgeTransLogLik", result.getEdgePredTransLogLik());
    jsonData.put("measurementLogLik", result.getMeasurementPredLogLik());
    return jsonData;
  }
  
  @JsonSerialize
  public Double getDistanceFromPreviousState() {
    return this.state.getDistanceFromPreviousState();
  }

  @JsonSerialize
  public String getEspgCode() {
    return this.state.getObservation().getObsProjected()
        .epsgCode();
  }

  @JsonIgnore
  public VehicleTrackingFilter getFilter() {
    return filter;
  }

  @JsonSerialize
  public OsmSegmentWithVelocity getInferredEdge() {
    return inferredEdge;
  }

  //  @JsonSerialize
  //  public Coordinate getMajorAxisCoords() {
  //    return majorAxisCoords;
  //  }

  //  @JsonSerialize
  //  public Coordinate getMinorAxisCoords() {
  //    return minorAxisCoords;
  //  }

  @JsonSerialize
  public Coordinate getMeanCoords() {
    return meanCoords;
  }
  
  @JsonSerialize
  public Double getPathDirection() {
    return pathDirection;
  }

  @JsonSerialize
  public List<OsmSegmentWithVelocity> getPathSegments() {
    return pathSegments;
  }

  @JsonSerialize
  public Map<String, Object> getPathState() {
    return getJsonForPathState(state.getBelief());
  }

  @JsonIgnore
  public VehicleState getState() {
    return state;
  }

  @JsonSerialize
  public List<OsmSegmentWithVelocity> getTraveledSegments() {
    final List<OsmSegmentWithVelocity> traveled =
        Lists.newArrayList();
    for (final OsmSegmentWithVelocity segment : this.pathSegments) {
      if (segment.getId() == this.inferredEdge.getId()) {
        traveled.add(segment);
        break;
      } else {
        traveled.add(segment);
      }
    }
    return traveled;
  }

  @JsonIgnore
  protected static Map<String, Object>
      getJsonForInvWishart(InverseWishartDistribution dist) {
    final Map<String, Object> jsonData = Maps.newHashMap();
    jsonData.put("dof", dist.getDegreesOfFreedom());
    jsonData
        .put(
            "scale",
            ((gov.sandia.cognition.math.matrix.mtj.DenseVector) dist
                .getInverseScale().convertToVector())
                .getArray().clone());
    return jsonData;
  }

  @JsonIgnore
  protected static Map<String, Object> getJsonForPathState(
    PathState pathState) {
    final Map<String, Object> jsonResult =
        Maps.newHashMap();
    jsonResult
        .put(
            "state",
            ((gov.sandia.cognition.math.matrix.mtj.DenseVector) pathState
                .getGlobalState()).getArray().clone());
    jsonResult
        .put(
            "stateLoc",
            ((gov.sandia.cognition.math.matrix.mtj.DenseVector) AbstractRoadTrackingFilter
                .getOg().times(pathState.getGroundState()))
                .getArray().clone());
    jsonResult.put("edge", new OsmSegment(pathState
        .getEdge().getInferredEdge()));

    if (pathState instanceof PathStateBelief) {
      final PathStateBelief simplePathStateBelief =
          (PathStateBelief) pathState;
      jsonResult.put("covariance",
          ((AbstractMTJMatrix) simplePathStateBelief
              .getCovariance()).convertToVector()
              .getArray().clone());
    }
    return jsonResult;
  }

}