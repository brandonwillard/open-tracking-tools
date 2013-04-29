package org.opentrackingtools.distributions;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;
import gov.sandia.cognition.util.DefaultPair;
import gov.sandia.cognition.util.Pair;

import java.util.List;
import java.util.Random;

import org.jfree.data.function.NormalDistributionFunction2D;
import org.opentrackingtools.util.StatisticsUtil;
import org.opentrackingtools.util.SvdMatrix;

import umontreal.iro.lecuyer.probdist.ContinuousDistribution;
import umontreal.iro.lecuyer.probdist.FoldedNormalDist;
import umontreal.iro.lecuyer.probdist.HalfNormalDist;
import umontreal.iro.lecuyer.probdist.NormalDist;
import umontreal.iro.lecuyer.probdist.TruncatedDist;
import umontreal.iro.lecuyer.probdistmulti.BiNormalDist;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;
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
  protected AdjMultivariateGaussian unTruncatedDist = new AdjMultivariateGaussian();
  
  public TruncatedRoadGaussian() {
  }
  
  public Range<Double> getDistanceRange() {
    return distanceRange;
  }

  public void setDistanceRange(Range<Double> distanceRange) {
    this.distanceRange = distanceRange;
  }

  public TruncatedRoadGaussian(MultivariateGaussian other) {
    super(other);
    if (other instanceof TruncatedRoadGaussian) {
      unTruncatedDist = ((TruncatedRoadGaussian)other).unTruncatedDist;
    } else {
      unTruncatedDist = new AdjMultivariateGaussian(other);
    }
  }

  public TruncatedRoadGaussian(MultivariateGaussian other,
    Range<Double> distanceRange) {
    super(other);
    if (other instanceof TruncatedRoadGaussian) {
      unTruncatedDist = ((TruncatedRoadGaussian)other).unTruncatedDist;
    } else {
      unTruncatedDist = new AdjMultivariateGaussian(other);
    }
    this.distanceRange = distanceRange;
    this.setMean(other.getMean());
    unTruncatedDist = new AdjMultivariateGaussian(other);
  }
  
  public TruncatedRoadGaussian(Vector mean, SvdMatrix svdMatrix) {
    Preconditions.checkArgument(mean.getDimensionality() == 2
        || mean.getDimensionality() == 4);
    Preconditions.checkArgument(svdMatrix.isSymmetric());
    Preconditions.checkArgument(
        svdMatrix.getNumRows() == 2
        || svdMatrix.getNumRows() == 4);
//    /*
//     * TODO perhaps we can remove this check.  especially
//     * if we expect different model designs.
//     */
//    Preconditions.checkArgument(svdMatrix.rank() == 1
//        || svdMatrix.rank() == 2);
    this.setMean(mean);
    this.setCovariance(svdMatrix);
  }
  
  public TruncatedRoadGaussian(Vector mean, SvdMatrix svdMatrix,
    Range<Double> distanceRange) {
    Preconditions.checkArgument(mean.getDimensionality() == 2
        || mean.getDimensionality() == 4);
    this.setMean(mean);
    this.setCovariance(svdMatrix);
    this.distanceRange = distanceRange;
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
    Preconditions.checkArgument(
        covariance.getNumRows() == 2
        || covariance.getNumRows() == 4);
    this.distanceTruncDist = null;
    super.setCovariance(covariance);
    if (this.unTruncatedDist != null)
      this.unTruncatedDist.setCovariance(covariance);
  }

  @Override
  public void setCovariance(Matrix covariance,
    double symmetryTolerance) {
    Preconditions.checkArgument(
        covariance.getNumRows() == 2
        || covariance.getNumRows() == 4);
    this.distanceTruncDist = null;
    super.setCovariance(covariance, symmetryTolerance);
    if (this.unTruncatedDist != null)
      this.unTruncatedDist.setCovariance(covariance, symmetryTolerance);
  }

  @Override
  public void setCovarianceInverse(Matrix covarianceInverse) {
    this.distanceTruncDist = null;
    super.setCovarianceInverse(covarianceInverse);
    if (this.unTruncatedDist != null)
      this.unTruncatedDist.setCovarianceInverse(covarianceInverse);
  }

  @Override
  public void setCovarianceInverse(Matrix covarianceInverse,
    double symmetryTolerance) {
    this.distanceTruncDist = null;
    super.setCovarianceInverse(covarianceInverse, symmetryTolerance);
    if (this.unTruncatedDist != null)
      this.unTruncatedDist.setCovarianceInverse(covarianceInverse, symmetryTolerance);
  }

  @Override
  public void setMean(Vector mean) {
    this.distanceTruncDist = null;
    super.setMean(this.truncateVector(mean));
    if (this.unTruncatedDist != null)
      this.unTruncatedDist.setMean(mean);
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

  public MultivariateGaussian getTruncatedDistribution() {
    final double rho = this.getCovariance().getElement(1,0)/
        (this.getCovariance().getElement(0,0) * this.getCovariance().getElement(1,1));
    final double c = 1d/Math.sqrt(1d-rho*rho);
    BiNormalDist biNormal = new BiNormalDist(
        this.getMean().getElement(0), this.getCovariance().getElement(0,0),
        this.getMean().getElement(1), this.getCovariance().getElement(1,1),
        rho); 
    
    final double b1 = this.getDistanceRange().lowerEndpoint();
    final double a1 = this.getDistanceRange().upperEndpoint();
    final double b2 = this.velocityRange.lowerEndpoint();
    final double a2 = this.velocityRange.upperEndpoint();
    
    final double L = biNormal.cdf(
        this.getDistanceRange().lowerEndpoint(), this.velocityRange.lowerEndpoint(), 
        this.getDistanceRange().upperEndpoint(), this.velocityRange.upperEndpoint());
    
    double Ed = 0d;
    if (!Double.isInfinite(a1))
      Ed += -NormalDist.density01(a1)
          * (NormalDist.cdf01((a2 - rho * a1)*c)
              - NormalDist.cdf01((b2 - rho * a1)*c));
    if (!Double.isInfinite(b1))
      Ed +=
         NormalDist.density01(b1)
          * (NormalDist.cdf01((a2 - rho * b1)*c)
              - NormalDist.cdf01((b1 - rho * b1)*c));
    if (!Double.isInfinite(a2))
      Ed +=
        - rho * NormalDist.density01(a2)
          * (NormalDist.cdf01((a1 - rho * a2)*c)
              - NormalDist.cdf01((b1 - rho * a2)*c));
    if (!Double.isInfinite(b2))
      Ed +=
         rho * NormalDist.density01(b2)
          * (NormalDist.cdf01((a1 - rho * b2)*c)
              - NormalDist.cdf01((b1 - rho * b2)*c));
    Ed /= L;
    
    double Ev = Double.NEGATIVE_INFINITY; 
//    double Ev = 0d; 
    double posSum = Double.NEGATIVE_INFINITY;
    double negSum = Double.NEGATIVE_INFINITY;
    if (!Double.isInfinite(a2))
      negSum =  
        // sign should be negative
        UnivariateGaussian.PDF.logEvaluate(a2, 0d, 1d)
          + LogMath.subtract(
              StatisticsUtil.normalCdf((a1 - rho * a2)*c, 0d, 1d, true),
              StatisticsUtil.normalCdf((b1 - rho * a2)*c, 0d, 1d, true));
//      Ev += -NormalDist.density01(a2)
//          * (NormalDist.cdf01((a1 - rho * a2)*c)
//              - NormalDist.cdf01((b1 - rho * a2)*c));
    if (!Double.isInfinite(b2))
      posSum = 
        UnivariateGaussian.PDF.logEvaluate(b2, 0d, 1d)
          + LogMath.subtract(
              StatisticsUtil.normalCdf((a1 - rho * b2)*c, 0d, 1d, true),
              StatisticsUtil.normalCdf((b2 - rho * b2)*c, 0d, 1d, true));
//      Ev += NormalDist.density01(b2)
//          * (NormalDist.cdf01((a1 - rho * b2)*c)
//              - NormalDist.cdf01((b2 - rho * b2)*c));
    if (!Double.isInfinite(a1))
      negSum = LogMath.add(negSum, 
        // sign should be negative
        Math.log(rho)
          + UnivariateGaussian.PDF.logEvaluate(a1, 0d, 1d)
          + LogMath.subtract(
              StatisticsUtil.normalCdf((a2 - rho * b1)*c, 0d, 1d, true),
              StatisticsUtil.normalCdf((b2 - rho * b1)*c, 0d, 1d, true)));
//      Ev += - rho * NormalDist.density01(a1)
//          * (NormalDist.cdf01((a2 - rho * a1)*c)
//              - NormalDist.cdf01((b2 - rho * a1)*c));
    if (!Double.isInfinite(b1))
      posSum = LogMath.add(posSum,
        Math.log(rho)
          + UnivariateGaussian.PDF.logEvaluate(b1, 0d, 1d)
          + LogMath.subtract(
              StatisticsUtil.normalCdf((a2 - rho * b1)*c, 0d, 1d, true),
              StatisticsUtil.normalCdf((b2 - rho * b1)*c, 0d, 1d, true)));
//      Ev += rho * NormalDist.density01(b1)
//          * (NormalDist.cdf01((a2 - rho * b1)*c)
//              - NormalDist.cdf01((b2 - rho * b1)*c));
    Ev = LogMath.subtract(posSum, negSum);
    Ev -= Math.log(L);
    Ev = Math.exp(Ev);
    
    MultivariateGaussian truncDist = new MultivariateGaussian();
    truncDist.setMean(VectorFactory.getDefault().copyArray(new double[] {
        Ed + biNormal.getMu1(), 
        Ev + biNormal.getMu2()}));
    
    return truncDist;
  }

  public AdjMultivariateGaussian getUnTruncatedDist() {
    return unTruncatedDist;
  }

  @Override
  public TruncatedRoadGaussian clone() {
    TruncatedRoadGaussian clone = (TruncatedRoadGaussian)super.clone();
    clone.unTruncatedDist = (AdjMultivariateGaussian) this.unTruncatedDist.clone();
    return clone;
  }
  
  
//  private double Fi(final int i) {
//    final int j = i == 0 ? 1 : 0;
//    final double ai;
//    final double bi;
//    final double aj;
//    final double bj; 
//    if (i == 0) {
//      ai = (this.getDistanceRange().lowerEndpoint() - this.getMean().getElement(0));
//      ai = (this.getDistanceRange().upperEndpoint() - this.getMean().getElement(0));
//      aj = (this.velocityRange.lowerEndpoint() - this.getMean().getElement(1));
//      bj = (this.velocityRange.upperEndpoint() - this.getMean().getElement(1));
//    } else {
//      aj = (this.getDistanceRange().lowerEndpoint() - this.getMean().getElement(0));
//      aj = (this.getDistanceRange().upperEndpoint() - this.getMean().getElement(0));
//      ai = (this.velocityRange.lowerEndpoint() - this.getMean().getElement(1));
//      bi = (this.velocityRange.upperEndpoint() - this.getMean().getElement(1));
//    }
//    final double sig_ii = this.getCovariance().getElement(i, i);
//    final double sig_ij = this.getCovariance().getElement(i, j);
//    final double sig_jj = this.getCovariance().getElement(j, j);
//    double val = 0d;
//    if (!Double.isInfinite(a1))
//      val += -NormalDist.density01(a1)
//          * (NormalDist.cdf01((a2 - rho * a1)*c)
//              - NormalDist.cdf01((b2 - rho * a1)*c));
//    if (!Double.isInfinite(b1))
//      val +=
//         NormalDist.density01(b1)
//          * (NormalDist.cdf01((a2 - rho * b1)*c)
//              - NormalDist.cdf01((b1 - rho * b1)*c));
//    if (!Double.isInfinite(a2))
//      val +=
//        - rho * NormalDist.density01(a2)
//          * (NormalDist.cdf01((a1 - rho * a2)*c)
//              - NormalDist.cdf01((b1 - rho * a2)*c));
//    if (!Double.isInfinite(b2))
//      val +=
//         rho * NormalDist.density01(b2)
//          * (NormalDist.cdf01((a1 - rho * b2)*c)
//              - NormalDist.cdf01((b1 - rho * b2)*c));
//    return val;
//  }
}
