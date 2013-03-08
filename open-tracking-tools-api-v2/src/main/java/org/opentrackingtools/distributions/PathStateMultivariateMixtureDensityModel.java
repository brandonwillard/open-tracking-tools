package org.opentrackingtools.distributions;

import gov.sandia.cognition.math.RingAccumulator;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.ClosedFormComputableDistribution;
import gov.sandia.cognition.statistics.ProbabilityDensityFunction;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.statistics.distribution.LinearMixtureModel;
import gov.sandia.cognition.util.ObjectUtil;

import java.util.Arrays;
import java.util.Collection;

import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathState;

/**
 * Copy of MultivariateMixtureDensityModel for path states
 */
public class PathStateMultivariateMixtureDensityModel<DistributionType extends ClosedFormComputableDistribution<PathState>>
    extends LinearMixtureModel<PathState, DistributionType> implements
    ClosedFormComputableDistribution<PathState> {

  private static final long serialVersionUID = 5143671580377145903L;

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((path == null) ? 0 : path.hashCode());
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj)
      return true;
    if (obj == null)
      return false;
    if (getClass() != obj.getClass())
      return false;
    PathStateMultivariateMixtureDensityModel<? extends DistributionType> other =
        (PathStateMultivariateMixtureDensityModel<? extends DistributionType>) obj;
    if (path == null) {
      if (other.path != null)
        return false;
    } else if (!path.equals(other.path))
      return false;
    return true;
  }

  protected Path path;

  public PathStateMultivariateMixtureDensityModel(Path path,
    Collection<? extends DistributionType> distributions) {
    this(path, distributions, null);
  }

  public PathStateMultivariateMixtureDensityModel(Path path,
    Collection<? extends DistributionType> distributions,
    double[] priorWeights) {
    super(distributions, priorWeights);
    this.path = path;
  }

  public PathStateMultivariateMixtureDensityModel(
    PathStateMultivariateMixtureDensityModel<? extends DistributionType> other) {
    this(other.path, ObjectUtil.cloneSmartElementsAsArrayList(other
        .getDistributions()), ObjectUtil.deepCopy(other
        .getPriorWeights()));
  }

  @Override
  public PathStateMultivariateMixtureDensityModel<DistributionType>
      clone() {
    PathStateMultivariateMixtureDensityModel<DistributionType> clone =
        (PathStateMultivariateMixtureDensityModel<DistributionType>) super
            .clone();
    clone.path = ObjectUtil.cloneSmart(this.path);
    
    return clone;
  }

  @Override
  public PathState getMean() {

    RingAccumulator<Vector> mean = new RingAccumulator<Vector>();
    final int K = this.getDistributionCount();
    for (int k = 0; k < K; k++) {
      mean.accumulate(this.getDistributions().get(k).getMean()
          .scale(this.getPriorWeights()[k]));
    }

    return new PathState(path, mean.getSum().scale(
        1.0 / this.getPriorWeightSum()));

  }

  @Override
  public Vector convertToVector() {
    return VectorFactory.getDefault().copyArray(
        this.getPriorWeights());
  }

  @Override
  public void convertFromVector(Vector parameters) {
    parameters
        .assertDimensionalityEquals(this.getDistributionCount());
    for (int k = 0; k < parameters.getDimensionality(); k++) {
      this.priorWeights[k] = parameters.getElement(k);
    }
  }

  @Override
  public
      PathStateMultivariateMixtureDensityModel.PDF<DistributionType>
      getProbabilityFunction() {
    return new PathStateMultivariateMixtureDensityModel.PDF<DistributionType>(
        this);
  }

  /**
   * PDF of the MultivariateMixtureDensityModel
   * 
   * @param <DistributionType>
   *          Type of Distribution in the mixture
   */
  public static class PDF<DistributionType extends ClosedFormComputableDistribution<PathState>>
      extends
      PathStateMultivariateMixtureDensityModel<DistributionType>
      implements ProbabilityDensityFunction<PathState> {

    private static final long serialVersionUID = -715039776358524176L;

    public PDF(Path path, Collection<? extends DistributionType> distributions) {
      super(path, distributions);
    }

    public PDF(Path path, Collection<? extends DistributionType> distributions,
      double[] priorWeights) {
      super(path, distributions, priorWeights);
    }

    public PDF(
      PathStateMultivariateMixtureDensityModel<? extends DistributionType> other) {
      super(other);
    }

    @Override
    public
        PathStateMultivariateMixtureDensityModel.PDF<DistributionType>
        getProbabilityFunction() {
      return this;
    }

    @Override
    public double logEvaluate(PathState input) {
      return Math.log(this.evaluate(input));
    }

    @Override
    public Double evaluate(PathState input) {
      double sum = 0.0;
      final int K = this.getDistributionCount();
      for (int k = 0; k < K; k++) {
        ProbabilityFunction<PathState> pdf =
            this.getDistributions().get(k).getProbabilityFunction();
        sum += pdf.evaluate(input) * this.priorWeights[k];
      }

      return sum / this.getPriorWeightSum();
    }

    /**
     * Computes the probability distribution that the input was generated by the
     * underlying distributions
     * 
     * @param input
     *          Input to consider
     * @return probability distribution that the input was generated by the
     *         underlying distributions
     */
    public double[] computeRandomVariableProbabilities(PathState input) {
      int K = this.getDistributionCount();
      double[] likelihoods =
          this.computeRandomVariableLikelihoods(input);
      double sum = 0.0;
      for (int k = 0; k < K; k++) {
        sum += likelihoods[k];
      }
      if (sum <= 0.0) {
        Arrays.fill(likelihoods, 1.0 / K);
      }

      sum = 0.0;
      for (int k = 0; k < K; k++) {
        likelihoods[k] *= this.priorWeights[k];
        sum += likelihoods[k];
      }
      if (sum <= 0.0) {
        Arrays.fill(likelihoods, 1.0 / K);
        sum = 1.0;
      }
      for (int k = 0; k < K; k++) {
        likelihoods[k] /= sum;
      }

      return likelihoods;

    }

    public double[] computeRandomVariableLikelihoods(PathState input) {

      int K = this.getDistributionCount();
      double[] likelihoods = new double[K];
      for (int k = 0; k < K; k++) {
        ProbabilityFunction<PathState> pdf =
            this.getDistributions().get(k).getProbabilityFunction();
        likelihoods[k] = pdf.evaluate(input);
      }

      return likelihoods;
    }

    public int getMostLikelyRandomVariable(PathState input) {

      double[] probabilities =
          this.computeRandomVariableProbabilities(input);
      int bestIndex = 0;
      double bestProbability = probabilities[0];
      for (int i = 1; i < probabilities.length; i++) {
        double prob = probabilities[i];
        if (bestProbability < prob) {
          bestProbability = prob;
          bestIndex = i;
        }
      }

      return bestIndex;

    }

  }

}
