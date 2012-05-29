package inference;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrixFactoryMTJ;
import gov.sandia.cognition.math.matrix.mtj.decomposition.EigenDecompositionRightMTJ;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.math.UnivariateStatisticsUtil;

import java.util.Date;
import java.util.List;

import models.InferenceInstance;

import org.openplans.tools.tracking.impl.InferredGraph.InferredEdge;
import org.openplans.tools.tracking.impl.InferredGraph;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.PathEdge;
import org.openplans.tools.tracking.impl.StandardRoadTrackingFilter;
import org.openplans.tools.tracking.impl.util.GeoUtils;

import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;

import controllers.Api;

public class InferenceResultRecord {

  private final String time;
  private final double originalLat;
  private final double originalLon;
  private final Double kfMeanLat;
  private final Double kfMeanLon;
  private final Double kfMajorLat;
  private final Double kfMajorLon;
  private final Double kfMinorLat;
  private final Double kfMinorLon;
  private final List<Double[]> graphSegmentIds;

  private InferenceResultRecord(long time, double originalLat,
    double originalLon, Double kfMeanLat, Double kfMeanLon, Double kfMajorLat,
    Double kfMajorLon, Double kfMinorLat, Double kfMinorLon,
    List<Double[]> idScaleList) {
    this.time = Api.sdf.format(new Date(time));
    this.originalLat = originalLat;
    this.originalLon = originalLon;
    this.kfMeanLat = kfMeanLat;
    this.kfMeanLon = kfMeanLon;
    this.kfMajorLat = kfMajorLat;
    this.kfMajorLon = kfMajorLon;
    this.kfMinorLat = kfMinorLat;
    this.kfMinorLon = kfMinorLon;
    this.graphSegmentIds = idScaleList;
  }

  public List<Double[]> getGraphSegmentIds() {
    return graphSegmentIds;
  }

  public Double getKfMajorLat() {
    return kfMajorLat;
  }

  public Double getKfMajorLon() {
    return kfMajorLon;
  }

  public Double getKfMeanLat() {
    return kfMeanLat;
  }

  public Double getKfMeanLon() {
    return kfMeanLon;
  }

  public Double getKfMinorLat() {
    return kfMinorLat;
  }

  public Double getKfMinorLon() {
    return kfMinorLon;
  }

  public double getOriginalLat() {
    return originalLat;
  }

  public double getOriginalLon() {
    return originalLon;
  }

  public String getTime() {
    return time;
  }

  public static InferenceResultRecord createInferenceResultRecord(
    Observation observation, InferenceInstance inferenceInstance) {
    MultivariateGaussian belief = inferenceInstance.getBestState().getGroundOnlyBelief();
    List<PathEdge> edges = inferenceInstance.getBestState().getPath().getEdges();
    return createInferenceResultRecord(observation, belief, edges);
  }
  
  public static InferenceResultRecord createInferenceResultRecord(
    Observation observation, final MultivariateGaussian belief, List<PathEdge> path) {

    if (belief != null) {
      /*
       * The last edge of the path should correspond to the current edge,
       * and the belief should be adjusted to the start of that edge.
       */
      final PathEdge currentEdge = PathEdge.getEdge(Iterables.getLast(path).getInferredEdge(), 0d);
      final MultivariateGaussian gbelief = belief.clone();
      StandardRoadTrackingFilter.convertToGroundBelief(gbelief, currentEdge);
      final Matrix O = StandardRoadTrackingFilter.getGroundObservationMatrix();

      final Vector infMean = O.times(gbelief.getMean().clone());

      final Vector minorAxis;
      final Vector majorAxis;
      if (currentEdge == PathEdge.getEmptyPathEdge()) {
        /*-
         * TODO only implemented for off-road
         * FIXME results look fishy
         */
        final EigenDecompositionRightMTJ decomp = EigenDecompositionRightMTJ
            .create(DenseMatrixFactoryMTJ.INSTANCE.copyMatrix( gbelief.getCovariance()) );
        
        final Matrix Shalf = MatrixFactory.getDefault().createIdentity(2, 2);
        Shalf.setElement(0, 0, Math.sqrt(decomp.getEigenValue(0).getRealPart()));
        Shalf.setElement(1, 1, Math.sqrt(decomp.getEigenValue(1).getRealPart()));
        majorAxis = infMean.plus(O.times(decomp.getEigenVectorsRealPart().getColumn(0))
            .times(Shalf).scale(1.98));
        minorAxis = infMean.plus(O.times(decomp.getEigenVectorsRealPart().getColumn(1))
            .times(Shalf).scale(1.98));
      } else {
        majorAxis = infMean;
        minorAxis = infMean;
      }

      final Coordinate kfMean = GeoUtils.convertToLatLon(infMean);
      final Coordinate kfMajor = GeoUtils.convertToLatLon(majorAxis);
      final Coordinate kfMinor = GeoUtils.convertToLatLon(minorAxis);
      
      List<Double[]> idScaleList = Lists.newArrayList();
      
      for (PathEdge edge : path) {
        if (edge == PathEdge.getEmptyPathEdge())
          continue;
        /*
         * FIXME TODO we should probably be using the edge convolutions at each step.
         */
        double mean = edge.getInferredEdge().getVelocityPrecisionDist().getLocation();
        double edgeId = edge.getInferredEdge().getEdgeId() != null ? 
           (double) edge.getInferredEdge().getEdgeId() : -1d;
        idScaleList.add(new Double[] {edgeId, mean});
      }

      return new InferenceResultRecord(observation.getTimestamp().getTime(),
          observation.getObsCoords().x, observation.getObsCoords().y,
          kfMean.x, kfMean.y, 
          kfMajor.x, kfMajor.y, 
          kfMinor.x, kfMinor.y,
          idScaleList);
    }

    return new InferenceResultRecord(observation.getTimestamp().getTime(),
        observation.getObsCoords().x, observation.getObsCoords().y,
        null, null, 
        null, null, 
        null, null,
        null);
  }

}
