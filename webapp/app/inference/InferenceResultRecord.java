package inference;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
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
  private final double kfMeanLat;
  private final double kfMeanLon;
  private final double kfMajorLat;
  private final double kfMajorLon;
  private final double kfMinorLat;
  private final double kfMinorLon;
  private final List<Double[]> graphSegmentIds;

  private InferenceResultRecord(long time, double originalLat,
    double originalLon, double kfMeanLat, double kfMeanLon, double kfMajorLat,
    double kfMajorLon, double kfMinorLat, double kfMinorLon,
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

  public double getKfMajorLat() {
    return kfMajorLat;
  }

  public double getKfMajorLon() {
    return kfMajorLon;
  }

  public double getKfMeanLat() {
    return kfMeanLat;
  }

  public double getKfMeanLon() {
    return kfMeanLon;
  }

  public double getKfMinorLat() {
    return kfMinorLat;
  }

  public double getKfMinorLon() {
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
    Observation observation, MultivariateGaussian belief, List<PathEdge> path) {

    if (belief != null) {
      /*
       * The last edge of the path should correspond to the current edge,
       * and the belief should be adjusted to the start of that edge.
       */
      final PathEdge currentEdge = PathEdge.getEdge(Iterables.getLast(path).getInferredEdge(), 0d);
      StandardRoadTrackingFilter.convertToGroundBelief(belief, currentEdge);
      final Matrix O = StandardRoadTrackingFilter.getGroundObservationMatrix();

      final Vector infMean = O.times(belief.getMean().clone());
      final DenseMatrix covar = (DenseMatrix) belief.getCovariance();

      final EigenDecompositionRightMTJ decomp = EigenDecompositionRightMTJ
          .create(covar);
      final Matrix Shalf = MatrixFactory.getDefault().createIdentity(2, 2);
      Shalf.setElement(0, 0, Math.sqrt(decomp.getEigenValue(0).getRealPart()));
      Shalf.setElement(1, 1, Math.sqrt(decomp.getEigenValue(1).getRealPart()));
      final Vector majorAxis = infMean.plus(O.times(decomp.getEigenVectorsRealPart().getColumn(0))
          .times(Shalf).scale(1.98));
      final Vector minorAxis = infMean.plus(O.times(decomp.getEigenVectorsRealPart().getColumn(1))
          .times(Shalf).scale(1.98));

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
        idScaleList.add(new Double[] {(double) edge.getInferredEdge().getEdgeId(), mean});
      }

      return new InferenceResultRecord(observation.getTimestamp().getTime(),
          observation.getObsCoords().y, observation.getObsCoords().x,
          kfMean.y, kfMean.x, 
          kfMajor.y, kfMajor.x, 
          kfMinor.y, kfMinor.x,
          idScaleList);
    }

    return null;
  }

}
