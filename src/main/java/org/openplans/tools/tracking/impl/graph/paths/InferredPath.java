package org.openplans.tools.tracking.impl.graph.paths;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;
import gov.sandia.cognition.util.DefaultPair;
import gov.sandia.cognition.util.Pair;

import java.util.List;
import java.util.Map;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.statistics.StatisticsUtil;
import org.openplans.tools.tracking.impl.statistics.WrappedWeightedValue;
import org.openplans.tools.tracking.impl.statistics.filters.StandardRoadTrackingFilter;

import com.google.common.annotations.Beta;
import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

/**
 * Inferred paths are collections of PathEdges that track the distance traveled
 * and the direction (by sign)
 * 
 * @author bwillard
 * 
 */
public class InferredPath implements Comparable<InferredPath> {

  public static class EdgePredictiveResults {
    final WrappedWeightedValue<MultivariateGaussian> weightedPredictiveDist;
    final Double edgeMarginalLikelihood;

    public EdgePredictiveResults(
      WrappedWeightedValue<MultivariateGaussian> weightedPredictiveDist,
      Double edgeMarginalLikelihood) {
      this.weightedPredictiveDist = weightedPredictiveDist;
      this.edgeMarginalLikelihood = edgeMarginalLikelihood;
    }

    public Double getEdgeMarginalLikelihood() {
      return edgeMarginalLikelihood;
    }

    public WrappedWeightedValue<MultivariateGaussian>
        getWeightedPredictiveDist() {
      return weightedPredictiveDist;
    }

  }

  private final ImmutableList<PathEdge> edges;
  private final Double totalPathDistance;

  public List<Integer> edgeIds = Lists.newArrayList();

  /*
   * These are the edges used in path finding.
   */
  private InferredEdge startSearchEdge;

  private InferredEdge endSearchEdge;

  /*
   * Note: single edges are considered forward
   */
  private Boolean isBackward = null;

  private final Geometry geometry;

  private static InferredPath emptyPath = new InferredPath();

  private static double edgeDistTolerance = 1e-4;

  private InferredPath() {
    this.edges = ImmutableList.of(PathEdge.getEmptyPathEdge());
    this.totalPathDistance = null;
    this.isBackward = null;
    this.geometry = null;
  }

  private InferredPath(ImmutableList<PathEdge> edges,
    boolean isBackward) {
    Preconditions.checkArgument(edges.size() > 0);
    this.edges = edges;
    this.isBackward = isBackward;

    PathEdge lastEdge = null;
    //    double absTotalDistance = 0d;
    final List<Coordinate> coords = Lists.newArrayList();
    for (final PathEdge edge : edges) {

      if (!edge.isEmptyEdge()) {
        if (isBackward) {
          assert (lastEdge == null || lastEdge.getInferredEdge()
              .getStartVertex()
              .equals(edge.getInferredEdge().getEndVertex()));
        } else {
          assert (lastEdge == null || lastEdge.getInferredEdge()
              .getEndVertex()
              .equals(edge.getInferredEdge().getStartVertex()));

        }

        final Geometry geom = edge.getInferredEdge().getGeometry();
        if (geom.getLength() > 1e-4) {
          final Coordinate[] theseCoords =
              isBackward ? geom.reverse().getCoordinates() : geom
                  .getCoordinates();
          final int startIdx = coords.size() == 0 ? 0 : 1;
          for (int i = startIdx; i < theseCoords.length; i++) {
            if (i == 0
                || !theseCoords[i]
                    .equals(coords.get(coords.size() - 1)))
              coords.add(theseCoords[i]);
          }
          edgeIds.add(edge.getInferredEdge().getEdgeId());
        }
      }

      lastEdge = edge;
    }

    if (edges.size() > 1) {
      this.geometry =
          JTSFactoryFinder.getGeometryFactory().createLineString(
              coords.toArray(new Coordinate[coords.size()]));
    } else {
      this.geometry = edges.get(0).getInferredEdge().getGeometry();
    }

    final double direction = isBackward ? -1d : 1d;
    this.totalPathDistance = direction * this.geometry.getLength();
  }

  private InferredPath(InferredEdge inferredEdge) {
    Preconditions.checkArgument(!inferredEdge.isEmptyEdge());
    this.edges =
        ImmutableList.of(PathEdge.getEdge(inferredEdge, 0d, false));
    this.totalPathDistance = inferredEdge.getLength();
    this.isBackward = Boolean.FALSE;
    this.edgeIds.add(inferredEdge.getEdgeId());
    this.geometry = inferredEdge.getGeometry();
  }

  private InferredPath(PathEdge edge) {
    Preconditions.checkArgument(!edge.isEmptyEdge());
    Preconditions.checkArgument(edge.getDistToStartOfEdge() == 0d);
    this.edges = ImmutableList.of(edge);
    this.totalPathDistance = edge.getInferredEdge().getLength();
    this.isBackward = Boolean.FALSE;
    this.edgeIds.add(edge.getInferredEdge().getEdgeId());
    this.geometry = edge.getInferredEdge().getGeometry();
  }

  private MultivariateGaussian
      calcBeliefPrediction(VehicleState state) {
    /*-
     * A prior predictive is created for every path, since, in some instances,
     * we need to project onto an edge and then predict movement.
     */
    final MultivariateGaussian beliefPrediction =
        state.getBelief().clone();

    /*
     * Convert to this path's direction
     */
    if (!state.getInferredEdge().isEmptyEdge() && !this.isEmptyPath())
      this.normalizeToPath(beliefPrediction, state.getInferredEdge());

    final StandardRoadTrackingFilter filter =
        state.getMovementFilter();

    filter.predict(beliefPrediction, this.getEdges().get(0),
        PathEdge.getEdge(state.getInferredEdge()));

    return beliefPrediction;

  }

  @Override
  public int compareTo(InferredPath o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(this.edges.toArray(), o.edges.toArray());
    return comparator.toComparison();
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
    final InferredPath other = (InferredPath) obj;
    if (edges == null) {
      if (other.edges != null) {
        return false;
      }
    } else if (!edges.equals(other.edges)) {
      return false;
    }
    return true;
  }

  public PathEdge getEdgeForDistance(double distance, boolean clamp) {
    final double direction = Math.signum(totalPathDistance);
    if (direction * distance > Math.abs(totalPathDistance)) {
      return clamp ? Iterables.getLast(edges) : null;
    } else if (direction * distance < 0d) {
      return clamp ? Iterables.getFirst(edges, null) : null;
    }

    for (final PathEdge edge : edges) {
      if (edge.isOnEdge(distance))
        return edge;
    }

    assert false;

    return null;
  }

  /**
   * XXX: the state must have a prior predictive mean.
   * 
   * @param obs
   * @param state
   * @param edgeToPreBeliefAndLogLik
   * @return
   */
  public
      InferredPathEntry
      getPredictiveLogLikelihood(
        Observation obs,
        VehicleState state,
        Map<Pair<PathEdge, Boolean>, EdgePredictiveResults> edgeToPreBeliefAndLogLik) {

    /*
     * We allow transitions from off-road onto a path, and vice-versa.  
     * Otherwise, we require that the first edge of the path is the edge of the
     * current state.
     */
    if (!(state.getInferredEdge().isEmptyEdge() || this.isEmptyPath() || state
        .getInferredEdge()
        .getGeometry()
        .equals(
            Iterables.getFirst(this.getEdges(), null)
                .getInferredEdge().getGeometry())))
      return null;

    /*
     * Lazily load this so we don't repeat work for dups.
     */
    MultivariateGaussian beliefPrediction = null;
    PathEdge prevEdge = PathEdge.getEdge(state.getInferredEdge());
    final StandardRoadTrackingFilter filter =
        state.getMovementFilter();

    double pathLogLik = Double.NEGATIVE_INFINITY;
    double edgePredMarginalTotalLik = Double.NEGATIVE_INFINITY;

    final List<WrappedWeightedValue<PathEdge>> weightedPathEdges =
        Lists.newArrayList();
    for (final PathEdge edge : this.getEdges()) {

      final Pair<PathEdge, Boolean> directionalEdge =
          new DefaultPair<PathEdge, Boolean>(edge, this.isBackward);
      final double localLogLik;
      final double edgePredMarginalLogLik;
      final PathEdge edgeOfLocPrediction;

      if (edgeToPreBeliefAndLogLik.containsKey(edge)) {
        final EdgePredictiveResults edgeResults =
            edgeToPreBeliefAndLogLik.get(edge);
        if (edgeResults == null)
          continue;

        localLogLik =
            edgeResults.getWeightedPredictiveDist().getWeight();
        edgePredMarginalLogLik =
            edgeResults.getEdgeMarginalLikelihood();
      } else {
        /*
         * If we're going off-road, then pass the edge we used to be on.
         */
        if (beliefPrediction == null) {
          beliefPrediction = this.calcBeliefPrediction(state);
        }

        final MultivariateGaussian locationPrediction =
            beliefPrediction.clone();
        if (edge.isEmptyEdge()) {
          // TODO meh?
          edgePredMarginalLogLik = 0d;
          edgeOfLocPrediction = edge;
        } else {
          predict(locationPrediction, obs, edge);
          edgePredMarginalLogLik =
              marginalPredictiveLogLikelihood(edge, beliefPrediction);
          /*
           * Since we don't truncate to the considered edge when producing
           * predicted locations, we just check if that the predicted location
           * is on the edge.  If it isn't we drop this possibility. 
           */
          if (!edge.equals(this.getEdgeForDistance(locationPrediction
              .getMean().getElement(0), false))) {
            edgeOfLocPrediction = null;
          } else {
            edgeOfLocPrediction = edge;
          }

        }

        if (edgeOfLocPrediction != null) {
          final double edgePredTransLogLik =
              state.getEdgeTransitionDist().predictiveLogLikelihood(
                  prevEdge.getInferredEdge(),
                  edgeOfLocPrediction.getInferredEdge());

          final double localPosVelPredLogLik =
              filter.logLikelihood(obs.getProjectedPoint(),
                  locationPrediction, edgeOfLocPrediction);

          assert !Double.isNaN(edgePredMarginalLogLik);
          assert !Double.isNaN(edgePredTransLogLik);
          assert !Double.isNaN(localPosVelPredLogLik);

          localLogLik =
              edgePredMarginalLogLik + edgePredTransLogLik
                  + localPosVelPredLogLik;
        } else {
          /*
           * If the result is not on the path, then stop evaluating
           * it.  This can happen when the projected location is in
           * the opposite direction, by a lot.  We really don't want
           * to evaluate edges that are even further away, since all
           * that will end up doing is eventually putting us on the
           * path, but with a very small likelihood and an extremely
           * skewed result.
           */
          // let's stop ourselves from evaluating this again
          edgeToPreBeliefAndLogLik.put(directionalEdge, null);
          continue;
        }

        /*
         * If we hit results with numerically zero likelihood, then
         * stop, because it's not going to get better from here.
         */
        if (Double.isInfinite(localLogLik)) {
          // let's stop ourselves from evaluating this again
          edgeToPreBeliefAndLogLik.put(directionalEdge, null);
          break;
        }

        /*
         * We're only going to deal with the terminating edge for now.
         */
        edgeToPreBeliefAndLogLik.put(directionalEdge,
            new EdgePredictiveResults(
                new WrappedWeightedValue<MultivariateGaussian>(
                    locationPrediction.clone(), localLogLik),
                edgePredMarginalLogLik));
      }

      /*
       * Add likelihood for this edge to the path total
       */
      weightedPathEdges.add(new WrappedWeightedValue<PathEdge>(edge,
          localLogLik));
      pathLogLik = LogMath.add(pathLogLik, localLogLik);
      edgePredMarginalTotalLik =
          LogMath.add(edgePredMarginalTotalLik,
              edgePredMarginalLogLik);

      assert !Double.isNaN(edgePredMarginalLogLik);

      prevEdge = edge;
    }

    if (Double.isInfinite(pathLogLik)
        && Double.isInfinite(edgePredMarginalTotalLik))
      return null;

    /*
     * Normalize with respect to the edges' predictive marginal 
     */
    if (!this.isEmptyPath())
      pathLogLik = pathLogLik - edgePredMarginalTotalLik;

    assert !Double.isNaN(pathLogLik);

    return new InferredPathEntry(this, edgeToPreBeliefAndLogLik,
        filter, weightedPathEdges, pathLogLik);
  }

  @Beta
  private double getTruncatedMean(final double origMean,
    double stdDev, PathEdge edge) {
    final double direction = edge.isBackward() ? -1d : 1d;
    final double mean =
        (Math.signum(origMean) * direction) * Math.abs(origMean);
    final double startDistance =
        Math.abs(edge.getDistToStartOfEdge());
    final double endDistance =
        edge.getInferredEdge().getLength() + startDistance;

    final double tt1 =
        UnivariateGaussian.PDF.logEvaluate(startDistance, mean,
            stdDev * stdDev);
    final double tt2 =
        UnivariateGaussian.PDF.logEvaluate(endDistance, mean, stdDev
            * stdDev);
    final double t1 =
        LogMath
            .subtract(tt1 > tt2 ? tt1 : tt2, tt1 > tt2 ? tt2 : tt1);

    final double t2 =
        LogMath
            .subtract(StatisticsUtil.normalCdf(endDistance, mean,
                stdDev, true), StatisticsUtil.normalCdf(
                startDistance, mean, stdDev, true));

    final double d2 = Math.log(stdDev) + t1 - t2;
    final double tmean = mean + (tt2 > tt1 ? -1d : 1d) * Math.exp(d2);

    return direction * tmean;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime * result + ((edges == null) ? 0 : edges.hashCode());
    return result;
  }

  public boolean isEmptyPath() {
    return this == emptyPath;
  }

  public boolean isOnPath(double distance) {

    Preconditions.checkState(!isEmptyPath());

    final double direction = Math.signum(totalPathDistance);
    if (direction * distance - Math.abs(totalPathDistance) > edgeDistTolerance) {
      return false;
    } else if (direction * distance < 0d) {
      return false;
    }

    return true;
  }

  public double marginalPredictiveLogLikelihood(PathEdge edge,
    MultivariateGaussian beliefPrediction) {
    //    MultivariateGaussian beliefPrediction, double pathEdgeDistNormFactor) {
    Preconditions.checkArgument(beliefPrediction
        .getInputDimensionality() == 2);
    final Matrix Or = StandardRoadTrackingFilter.getOr();
    final double var =
        Or.times(beliefPrediction.getCovariance())
            .times(Or.transpose()).getElement(0, 0)
            + Math.pow(edge.getInferredEdge().getLength(), 2d)
            / Math.sqrt(12);
    final double mean =
        Or.times(beliefPrediction.getMean()).getElement(0);
    final double direction = Math.signum(totalPathDistance);

    final double evalPoint =
        (edge.getDistToStartOfEdge() + (edge.getDistToStartOfEdge() + direction
            * edge.getInferredEdge().getLength())) / 2d;

    final double result =
        UnivariateGaussian.PDF.logEvaluate(evalPoint, mean, var);
    //        + pathEdgeDistNormFactor; 

    return result;
  }

  public double marginalPredictiveLogLikelihoodOld(PathEdge edge,
    MultivariateGaussian beliefPrediction) {
    Preconditions.checkArgument(beliefPrediction
        .getInputDimensionality() == 2);
    final Matrix Or = StandardRoadTrackingFilter.getOr();
    final double stdDev =
        Math.sqrt(Or.times(beliefPrediction.getCovariance())
            .times(Or.transpose()).getElement(0, 0));
    final double mean =
        Or.times(beliefPrediction.getMean()).getElement(0);
    final double direction = Math.signum(totalPathDistance);
    final double distToEndOfEdge =
        direction * edge.getInferredEdge().getLength()
            + edge.getDistToStartOfEdge();
    final double startDistance =
        direction > 0d ? edge.getDistToStartOfEdge()
            : distToEndOfEdge;
    final double endDistance =
        direction > 0d ? distToEndOfEdge : edge
            .getDistToStartOfEdge();

    // FIXME use actual log calculations
    final double result =
        LogMath
            .subtract(StatisticsUtil.normalCdf(endDistance, mean,
                stdDev, true), StatisticsUtil.normalCdf(
                startDistance, mean, stdDev, true));

    return result;
  }

  /**
   * Converts location component of the mean to a location on this path, if any.
   * For example, negative positions on the current edge are turned into
   * positive positions, and if they extend past the edge in the negative
   * direction, then the remaining negative distance is used.
   * 
   * @param beliefPrediction
   */
  private void normalizeToPath(MultivariateGaussian beliefPrediction,
    InferredEdge presentEdge) {

    final InferredEdge edge = edges.get(0).getInferredEdge();
    final boolean pathEdgeIsReverse =
        presentEdge.getGeometry().reverse()
            .equalsExact(edge.getGeometry());

    /*
     * When the edges are in opposite directions, 
     */
    if (pathEdgeIsReverse) {
      final double currentLocation =
          beliefPrediction.getMean().getElement(0);
      final double newLocation;
      if (currentLocation == 0d)
        newLocation = edge.getLength();
      else
        newLocation =
            Math.signum(currentLocation)
                * (edge.getLength() - Math.abs(currentLocation));

      beliefPrediction.getMean().setElement(0, newLocation);
      beliefPrediction.getMean().setElement(1,
          -1d * beliefPrediction.getMean().getElement(1));
    }

    final double currentLocation =
        beliefPrediction.getMean().getElement(0);

    if (isBackward == Boolean.TRUE && currentLocation > 0d) {
      final double newLocation =
          Math.min(0, -edge.getLength() + currentLocation);
      beliefPrediction.getMean().setElement(0, newLocation);
    } else if (isBackward == Boolean.FALSE && currentLocation < 0d) {
      final double newLocation =
          Math.max(0, currentLocation + edge.getLength());
      beliefPrediction.getMean().setElement(0, newLocation);
    }

    assert this.isOnPath(beliefPrediction.getMean().getElement(0));

  }

  /**
   * This method truncates the given belief over the interval defined by this
   * edge.
   * 
   * @param belief
   * @param edge2
   */
  public void predict(MultivariateGaussian belief, Observation obs,
    PathEdge edge) {

    Preconditions.checkArgument(belief.getInputDimensionality() == 2);
    Preconditions.checkArgument(edges.contains(edge));

    /*-
     * TODO really, this should just be the truncated/conditional
     * mean and covariance for the given interval/edge
     */
    final Matrix Or = StandardRoadTrackingFilter.getOr();
    final double S =
        Or.times(belief.getCovariance()).times(Or.transpose())
            .getElement(0, 0)
            // + 1d;
            + Math
                .pow(
                    edge.getInferredEdge().getLength()
                        / Math.sqrt(12), 2);
    final Matrix W =
        belief.getCovariance().times(Or.transpose()).scale(1 / S);
    final Matrix R =
        belief.getCovariance().minus(W.times(W.transpose()).scale(S));

    final double direction = Math.signum(totalPathDistance);
    final double mean =
        (edge.getDistToStartOfEdge() + (edge.getDistToStartOfEdge() + direction
            * edge.getInferredEdge().getLength())) / 2d;

    final double e = mean - Or.times(belief.getMean()).getElement(0);
    final Vector a = belief.getMean().plus(W.getColumn(0).scale(e));

    belief.setMean(a);
    belief.setCovariance(R);
  }

  public void setEndSearchEdge(InferredEdge endEdge) {
    this.endSearchEdge = endEdge;
  }

  public void setIsBackward(Boolean isBackward) {
    this.isBackward = isBackward;
  }

  public void setStartSearchEdge(InferredEdge startEdge) {
    this.startSearchEdge = startEdge;
  }

  @Override
  public String toString() {
    if (this == emptyPath)
      return "InferredPath [empty path]";
    else
      return "InferredPath [edges=" + edgeIds
          + ", totalPathDistance=" + totalPathDistance + "]";
  }

  public static InferredPath getEmptyPath() {
    return emptyPath;
  }

  public static InferredPath
      getInferredPath(InferredEdge inferredEdge) {
    if (inferredEdge.isEmptyEdge())
      return emptyPath;
    else
      return new InferredPath(inferredEdge);
  }

  public static InferredPath getInferredPath(List<PathEdge> edges,
    boolean isBackward) {
    if (edges.size() == 1) {
      final PathEdge edge = Iterables.getOnlyElement(edges);
      if (edge.isEmptyEdge())
        return emptyPath;
      else
        return new InferredPath(edge);
    }
    return new InferredPath(ImmutableList.copyOf(edges), isBackward);
  }

  public static InferredPath getInferredPath(PathEdge pathEdge) {
    if (pathEdge.isEmptyEdge())
      return emptyPath;
    else
      return new InferredPath(pathEdge);
  }

  public ImmutableList<PathEdge> getEdges() {
    return edges;
  }

  public Double getTotalPathDistance() {
    return totalPathDistance;
  }

  public List<Integer> getEdgeIds() {
    return edgeIds;
  }

  public InferredEdge getStartSearchEdge() {
    return startSearchEdge;
  }

  public InferredEdge getEndSearchEdge() {
    return endSearchEdge;
  }

  public Boolean getIsBackward() {
    return isBackward;
  }

  public Geometry getGeometry() {
    return geometry;
  }

  public static double getEdgeDistTolerance() {
    return edgeDistTolerance;
  }

}
