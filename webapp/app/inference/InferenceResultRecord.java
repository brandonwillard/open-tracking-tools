package inference;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrixFactoryMTJ;
import gov.sandia.cognition.math.matrix.mtj.DenseVector;
import gov.sandia.cognition.math.matrix.mtj.decomposition.EigenDecompositionRightMTJ;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.Collections;
import java.util.Date;
import java.util.List;

import models.InferenceInstance;

import org.codehaus.jackson.annotate.JsonIgnore;
import org.codehaus.jackson.map.annotate.JsonSerialize;
import org.openplans.tools.tracking.impl.FilterInformation;
import org.openplans.tools.tracking.impl.InferredPath;
import org.openplans.tools.tracking.impl.InferredPathEntry;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.PathEdge;
import org.openplans.tools.tracking.impl.StandardRoadTrackingFilter;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.util.GeoUtils;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;

import controllers.Api;

public class InferenceResultRecord {

  private static class ResultSet {

    private final Coordinate meanCoords;
    private final Coordinate majorAxisCoords;
    private final Coordinate minorAxisCoords;
    private final List<Double[]> pathSegmentIds;
    private final VehicleState state;

    public ResultSet(VehicleState vehicleState, Coordinate meanCoords,
      Coordinate majorAxisCoords, Coordinate minorAxisCoords,
      List<Double[]> pathSegmentIds) {
      this.meanCoords = meanCoords;
      this.majorAxisCoords = majorAxisCoords;
      this.minorAxisCoords = minorAxisCoords;
      this.pathSegmentIds = pathSegmentIds;
      this.state = vehicleState;
    }

    @JsonSerialize
    public List<List<Integer>> getEvaluatedPaths() {
      List<List<Integer>> pathEdgeIds;
      if (state.getFilterInformation() != null) {
        pathEdgeIds = Lists.newArrayList();
        for (final InferredPathEntry pathEntry : state.getFilterInformation()
            .getEvaluatedPaths()) {
          final List<Integer> edgeIds = Lists.newArrayList();
          for (final PathEdge edge : pathEntry.getPath().getEdges()) {
            if (edge.getInferredEdge().getEdgeId() != null)
              edgeIds.add(edge.getInferredEdge().getEdgeId());
          }
          if (!edgeIds.isEmpty())
            pathEdgeIds.add(edgeIds);
        }
      } else {
        pathEdgeIds = Collections.emptyList();
      }
      return pathEdgeIds;
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
    public List<Double[]> getPathSegmentIds() {
      return pathSegmentIds;
    }

    @JsonIgnore
    public VehicleState getState() {
      return state;
    }

    @JsonSerialize
    public double[] getStateCovariance() {
      return ((DenseMatrix) state.getBelief().getCovariance())
          .convertToVector().getArray();
    }

    @JsonSerialize
    public double[] getStateMean() {
      return ((DenseVector) state.getBelief().getMean()).getArray();
    }

  }

  private final String time;

  private final Coordinate observedCoords;

  private final ResultSet actualResults;

  private final ResultSet infResults;

  public InferenceResultRecord(long time, Coordinate obsCoords,
    ResultSet actualResults, ResultSet infResults) {
    this.actualResults = actualResults;
    this.infResults = infResults;
    this.observedCoords = obsCoords;
    this.time = Api.sdf.format(new Date(time));
  }

  @JsonSerialize
  public ResultSet getActualResults() {
    return actualResults;
  }

  @JsonSerialize
  public ResultSet getInfResults() {
    return infResults;
  }

  @JsonSerialize
  public Coordinate getObservedCoords() {
    return observedCoords;
  }

  @JsonSerialize
  public String getTime() {
    return time;
  }

  public static InferenceResultRecord createInferenceResultRecord(
    Observation observation, InferenceInstance inferenceInstance) {
    return createInferenceResultRecord(observation, null,
        inferenceInstance.getBestState());
  }

  public static InferenceResultRecord createInferenceResultRecord(
    Observation observation, VehicleState actualState,
    VehicleState inferredState) {

    Preconditions.checkNotNull(observation);

    ResultSet actualResults = null;
    if (actualState != null) {
      actualResults = processVehicleStateResults(actualState);
    }

    ResultSet infResults = null;
    if (inferredState != null) {
      infResults = processVehicleStateResults(inferredState);
    }

    return new InferenceResultRecord(observation.getTimestamp().getTime(),
        observation.getObsCoords(), actualResults, infResults);
  }

  private static ResultSet processVehicleStateResults(VehicleState state) {

    /*
     * The last edge of the path should correspond to the current edge, and the
     * belief should be adjusted to the start of that edge.
     */
    final MultivariateGaussian belief = state.getBelief();

    final PathEdge currentEdge = PathEdge.getEdge(state.getInferredEdge(), 0d);
    final MultivariateGaussian gbelief = belief.clone();
    final Matrix O = StandardRoadTrackingFilter.getGroundObservationMatrix();
    final Vector mean;
    final Vector minorAxis;
    final Vector majorAxis;

    StandardRoadTrackingFilter.convertToGroundBelief(gbelief, currentEdge);

    mean = O.times(gbelief.getMean().clone());

    if (currentEdge == PathEdge.getEmptyPathEdge()) {
      /*-
       * TODO only implemented for off-road
       * FIXME results look fishy
       */
      final EigenDecompositionRightMTJ decomp = EigenDecompositionRightMTJ
          .create(DenseMatrixFactoryMTJ.INSTANCE.copyMatrix(gbelief
              .getCovariance()));

      final Matrix Shalf = MatrixFactory.getDefault().createIdentity(2, 2);
      Shalf.setElement(0, 0, Math.sqrt(decomp.getEigenValue(0).getRealPart()));
      Shalf.setElement(1, 1, Math.sqrt(decomp.getEigenValue(1).getRealPart()));
      majorAxis = mean.plus(O
          .times(decomp.getEigenVectorsRealPart().getColumn(0)).times(Shalf)
          .scale(1.98));
      minorAxis = mean.plus(O
          .times(decomp.getEigenVectorsRealPart().getColumn(1)).times(Shalf)
          .scale(1.98));
    } else {
      majorAxis = mean;
      minorAxis = mean;
    }

    final Coordinate meanCoords = GeoUtils.convertToLatLon(mean);
    final Coordinate majorAxisCoords = GeoUtils.convertToLatLon(majorAxis);
    final Coordinate minorAxisCoords = GeoUtils.convertToLatLon(minorAxis);

    final List<Double[]> pathSegmentIds = Lists.newArrayList();
    final FilterInformation info = state.getFilterInformation();
    if (info != null) {
      final InferredPath path = info.getPath();
      for (final PathEdge edge : path.getEdges()) {
        if (edge == PathEdge.getEmptyPathEdge())
          continue;
        /*
         * FIXME TODO we should probably be using the edge convolutions at each
         * step.
         */
        final double edgeMean = edge.getInferredEdge()
            .getVelocityPrecisionDist().getLocation();
        final double edgeId = edge.getInferredEdge().getEdgeId() != null ? (double) edge
            .getInferredEdge().getEdgeId() : -1d;
        pathSegmentIds.add(new Double[] { edgeId, edgeMean });
      }
    }

    return new ResultSet(state.clone(), meanCoords, majorAxisCoords,
        minorAxisCoords, pathSegmentIds);
  }

}
