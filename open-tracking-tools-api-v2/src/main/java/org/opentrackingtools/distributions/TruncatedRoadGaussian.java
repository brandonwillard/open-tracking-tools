package org.opentrackingtools.distributions;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;
import gov.sandia.cognition.util.DefaultPair;
import gov.sandia.cognition.util.Pair;

import java.util.Random;

import org.opentrackingtools.util.StatisticsUtil;

import umontreal.iro.lecuyer.probdist.ContinuousDistribution;
import umontreal.iro.lecuyer.probdist.FoldedNormalDist;
import umontreal.iro.lecuyer.probdist.HalfNormalDist;
import umontreal.iro.lecuyer.probdist.NormalDist;
import umontreal.iro.lecuyer.probdist.TruncatedDist;

import com.google.common.base.Preconditions;
import com.google.common.collect.Range;
import com.google.common.collect.Ranges;

/**
 * 
 * Truncated velocities when on-road, not when off-road.<br>
 * One can specify the distance component's truncation range, but
 * not the velocity (which is [0, Inf)).
 * 
 * @author bwillard
 * 
 */
public class TruncatedRoadGaussian extends AdjMultivariateGaussian {

  private static final long serialVersionUID = -7465667744835664792L;

  protected ContinuousDistribution distanceTruncDist;

  public static Range<Double> velocityRange = Ranges.closed(0d, Double.POSITIVE_INFINITY);
  protected Range<Double> distanceRange = Ranges.closed(0d, Double.POSITIVE_INFINITY);

  public TruncatedRoadGaussian() {
  }
  
  public Range<Double> getDistanceRange() {
    return distanceRange;
  }

  public void setDistanceRange(Range<Double> distanceRange) {
    this.distanceRange = distanceRange;
  }

  public TruncatedRoadGaussian(MultivariateGaussian other) {
    this(other.getMean(), other.getCovariance());
  }

  public TruncatedRoadGaussian(MultivariateGaussian other,
    Range<Double> distanceRange) {
    this(other.getMean(), other.getCovariance(), distanceRange);
  }
  
  public TruncatedRoadGaussian(Vector mean, Matrix covariance) {
    Preconditions.checkArgument(mean.getDimensionality() == 2
        || mean.getDimensionality() == 4);
    Preconditions.checkArgument(covariance.getNumColumns() == 2
        || covariance.getNumColumns() == 4);
    this.setMean(mean);
    this.setCovariance(covariance);
  }

  public TruncatedRoadGaussian(Vector mean, Matrix covariance,
    Range<Double> distanceRange) {
    Preconditions.checkArgument(mean.getDimensionality() == 2
        || mean.getDimensionality() == 4);
    Preconditions.checkArgument(covariance.getNumColumns() == 2
        || covariance.getNumColumns() == 4);
    this.distanceRange = distanceRange;
    this.setMean(mean);
    this.setCovariance(covariance);
  }

  @Override
  public double getLogLeadingCoefficient() {
    return super.getLogLeadingCoefficient();
  }
  
  public static class PDF extends MultivariateGaussian.PDF {
    
    // FIXME ewww!
    protected TruncatedRoadGaussian self;

    public PDF(TruncatedRoadGaussian other) {
      super(other);
      self = other;
    }

    @Override
    public Double evaluate(Vector input) {
      return Math.exp(super.logEvaluate(input));
    }

    @Override
    public double logEvaluate(Vector input) {
      if (input.getDimensionality() == 2) {
        
        if (!self.distanceRange.contains(input.getElement(0))
            || !velocityRange.contains(input.getElement(1)))
          return Double.NEGATIVE_INFINITY;
        
        /*
         * Knowing that anything road-state being evaluated was generated
         * from the same degenerate covariance structure, we only evaluate
         * the distance.  
         */
//        Pair<Double,Double> velTruncPair = this.self.getConditionalTruncatedVelocityParams(input.getElement(0));
        
        return UnivariateGaussian.PDF.logEvaluate(input.getElement(0), 
            this.self.getDistanceTruncatedDistribution().getMean(), 
            this.self.getDistanceTruncatedDistribution().getVariance());
//            + UnivariateGaussian.PDF.logEvaluate(input.getElement(1), velTruncPair.getFirst(), 
//                velTruncPair.getSecond());
        
      } else {
        return super.logEvaluate(input);
      }
    }
    
    
  }

  @Override
  public PDF getProbabilityFunction() {
    return new PDF(this);
  }

  public ContinuousDistribution getDistanceTruncatedDistribution() {
    if (this.getInputDimensionality() == 4)
      return null;
    
    if (this.distanceTruncDist == null) {
      if (distanceRange.lowerEndpoint() == 0d &&
          distanceRange.upperEndpoint() == Double.POSITIVE_INFINITY) {
        distanceTruncDist =
            new FoldedNormalDist(this.getMean()
                .getElement(0), Math.sqrt(this.getCovariance()
                .getElement(0, 0)));
      } else if (Double.compare(distanceRange.lowerEndpoint(), this.getMean().getElement(0)) == 0 &&
          distanceRange.upperEndpoint() == Double.POSITIVE_INFINITY) {
        distanceTruncDist =
            new HalfNormalDist(this.getMean()
                .getElement(0), Math.sqrt(this.getCovariance()
                .getElement(0, 0)));
      } else {
        distanceTruncDist =
            new TruncatedDist(new NormalDist(this.getMean()
                .getElement(0), Math.sqrt(this.getCovariance()
                .getElement(0, 0))), 
                distanceRange.lowerEndpoint(),
                distanceRange.upperEndpoint());
      }
    }
    return distanceTruncDist; 
  }

  public static double getTruncatedNormalMean(final double origMean,
    double stdDev, Range<Double> range) {
    final double mean = origMean;
    final double startDistance = range.lowerEndpoint();
    final double endDistance = range.upperEndpoint();

    final double logPhi_a =
        UnivariateGaussian.PDF.logEvaluate(startDistance,
            mean, stdDev * stdDev);
    final double logPhi_b =
        UnivariateGaussian.PDF.logEvaluate(endDistance,
            mean, stdDev * stdDev);
    double t1 =
        LogMath.subtract(logPhi_a, logPhi_b);
    double s1 = 1d;
    if (Double.isNaN(t1)) {
      t1 = LogMath.subtract(logPhi_b, logPhi_a);
      s1 = -1d;
    }
    
    final double Zp1 = StatisticsUtil.normalCdf(
            endDistance, mean, stdDev, true);
    final double Zp2 = StatisticsUtil.normalCdf(startDistance, mean,
                stdDev, true);
    double logZ =
        LogMath.subtract(Zp1,Zp2);
    double s2 = 1d;
    if (Double.isNaN(logZ)) {
      logZ = LogMath.subtract(Zp2,Zp1);
      s2 = -1d;
    }

    final double d2 = Math.log(stdDev) + t1 - logZ;
    final double tmean = mean + s1 * s2 * Math.exp(d2);

    return tmean;
  }
  
  /**
   * Performs Gibbs sampling for the bivariate truncated case.
   * @param random
   * @return
   */
  @Override
  public Vector sample(Random random) {
    final Vector sample;

    if (this.getMean().getDimensionality() <= 2) {
      final double truncDistanceSmpl =
          this.getDistanceTruncatedDistribution().inverseF(random.nextDouble());
      
      Pair<Double, Double> velMeanVar = getConditionalTruncatedVelocityParams(truncDistanceSmpl);
      sample = VectorFactory.getDefault().createVector2D(truncDistanceSmpl, velMeanVar.getFirst());
    } else {
      sample = super.sample(random);
    }

    return sample;
  }
  
  /**
   * Returns the velocity conditional mean and the *marginal* velocity variance.
   * @param truncDistanceSmpl
   * @return
   */
  public Pair<Double, Double> getConditionalTruncatedVelocityParams(
    double truncDistanceSmpl) {
    final double S22 = this.getCovariance().getElement(1, 1);
    final double truncVelocityMean;
//    final double truncVelocityVar = 0d;
//    if (velocityRange.upperEndpoint() == Double.POSITIVE_INFINITY) {
//     truncVelocityMean = FoldedNormalDist.getMean(this.getMean().getElement(1), Math.sqrt(S22));
//     truncVelocityVar = FoldedNormalDist.getVariance(this.getMean().getElement(1), Math.sqrt(S22));
//    } else {
//      final ContinuousDistribution trunDist = new TruncatedDist(
//          new NormalDist(this.getMean().getElement(1), Math.sqrt(S22)), 0d, velocityRange.upperEndpoint());
//      truncVelocityMean = trunDist.getMean();
//      truncVelocityVar = trunDist.getVariance();
//    }
    truncVelocityMean = getTruncatedNormalMean(this.getMean().getElement(1), 
        Math.sqrt(S22), velocityRange);
//    Preconditions.checkState(velocityRange.contains(truncVelocityMean));
    
    ContinuousDistribution distTruncDist = this.getDistanceTruncatedDistribution();
    final double distTruncMean = distTruncDist.getMean();
    final double distTruncVar = distTruncDist.getVariance();
    
    final double S12 = this.getCovariance().getElement(0, 1);
    final double S11 = distTruncVar;
    
//    final double distanceMeanTest = getTruncatedNormalMean(
//        this.getMean().getElement(0), 
//        this.getCovariance().getElement(0, 0), this.distanceRange);
//    final double velocityMeanTest = getTruncatedNormalMean(
//        this.getMean().getElement(1), 
//        this.getCovariance().getElement(1, 1), this.distanceRange);
//    final double testDiff = distTruncMean - distanceMeanTest;
    
    final double distanceMean = distTruncMean;
    
    double conditionalMean = truncVelocityMean 
        + S12/S11 * (truncDistanceSmpl - distanceMean);
    
    conditionalMean = Math.min(velocityRange.upperEndpoint(), Math.max(velocityRange.lowerEndpoint(), conditionalMean));
    
    Preconditions.checkState(velocityRange.contains(conditionalMean));
    
    return DefaultPair.create(conditionalMean, 0d);
  }

  @Override
  public void setCovariance(Matrix covariance) {
    this.distanceTruncDist = null;
    super.setCovariance(covariance);
  }

  @Override
  public void setCovariance(Matrix covariance,
    double symmetryTolerance) {
    this.distanceTruncDist = null;
    super.setCovariance(covariance, symmetryTolerance);
  }

  @Override
  public void setCovarianceInverse(Matrix covarianceInverse) {
    this.distanceTruncDist = null;
    super.setCovarianceInverse(covarianceInverse);
  }

  @Override
  public void setCovarianceInverse(Matrix covarianceInverse,
    double symmetryTolerance) {
    this.distanceTruncDist = null;
    super.setCovarianceInverse(covarianceInverse, symmetryTolerance);
  }

  @Override
  public void setCovSqrt(Matrix covSqrt) {
    this.distanceTruncDist = null;
    super.setCovSqrt(covSqrt);
  }

  @Override
  public void setMean(Vector mean) {
    this.distanceTruncDist = null;
    super.setMean(this.truncateVector(mean));
  }

  protected Vector truncateVector(Vector mean) {
    if (mean.getDimensionality() == 2 && this.distanceRange != null) {
      final Vector adjMean = mean.clone();
      
      adjMean.setElement(0, 
          Math.min(this.distanceRange.upperEndpoint(),
              Math.max(this.distanceRange.lowerEndpoint(), adjMean.getElement(0))));
      
      adjMean.setElement(1,
          Math.min(Double.POSITIVE_INFINITY,
              Math.max(0d, adjMean.getElement(1))));
      
      return adjMean;
    }
    return mean;
  }
}
