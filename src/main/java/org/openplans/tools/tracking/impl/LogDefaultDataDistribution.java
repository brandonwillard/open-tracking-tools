package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.factory.Factory;
import gov.sandia.cognition.learning.algorithm.AbstractBatchAndIncrementalLearner;
import gov.sandia.cognition.math.MathUtil;
import gov.sandia.cognition.math.MutableDouble;
import gov.sandia.cognition.math.matrix.DefaultInfiniteVector;
import gov.sandia.cognition.math.matrix.InfiniteVector;
import gov.sandia.cognition.statistics.AbstractDataDistribution;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.DistributionEstimator;
import gov.sandia.cognition.statistics.DistributionWeightedEstimator;
import gov.sandia.cognition.statistics.ProbabilityMassFunctionUtil;
import gov.sandia.cognition.util.AbstractCloneableSerializable;
import gov.sandia.cognition.util.ArgumentChecker;
import gov.sandia.cognition.util.WeightedValue;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Random;
import java.util.Set;

import org.openplans.tools.tracking.impl.statistics.StatisticsUtil;

import com.google.common.collect.HashMultiset;
import com.google.common.collect.Maps;
import com.google.common.collect.Multiset;

/**
 * A default implementation of {@code ScalarDataDistribution} that uses a
 * backing map, and keeps values in log-scale. It also keeps a count of
 * duplicate values. Copied from {@code DefaultDataDistribution}.
 * 
 * @param <KeyType>
 *          Type of Key in the distribution
 */
public class LogDefaultDataDistribution<KeyType> implements DataDistribution<KeyType> {

  /**
   * A factory for {@code LogDefaultDataDistribution} objects using some given
   * initial capacity for them.
   * 
   * @param <DataType>
   *          The type of data for the factory.
   */
  public static class DefaultFactory<DataType> extends
      AbstractCloneableSerializable implements
      Factory<LogDefaultDataDistribution<DataType>> {

    private static final long serialVersionUID = 681699655965182747L;

    /* 
     * The initial domain capacity. 
     */
    protected int initialDomainCapacity;

    /**
     * Creates a new {@code DefaultFactory} with a default initial domain
     * capacity.
     */
    public DefaultFactory() {
      this(DEFAULT_INITIAL_CAPACITY);
    }

    /**
     * Creates a new {@code DefaultFactory} with a given initial domain
     * capacity.
     * 
     * @param initialDomainCapacity
     *          The initial capacity for the domain. Must be positive.
     */
    public DefaultFactory(final int initialDomainCapacity) {
      super();

      this.setInitialDomainCapacity(initialDomainCapacity);
    }

    @Override
    public LogDefaultDataDistribution<DataType> create() {
      return new LogDefaultDataDistribution<DataType>(
          this.getInitialDomainCapacity());
    }

    /**
     * Gets the initial domain capacity.
     * 
     * @return The initial domain capacity. Must be positive.
     */
    public int getInitialDomainCapacity() {
      return this.initialDomainCapacity;
    }

    /**
     * Sets the initial domain capacity.
     * 
     * @param initialDomainCapacity
     *          The initial domain capacity. Must be positive.
     */
    public void setInitialDomainCapacity(
      final int initialDomainCapacity) {
      ArgumentChecker.assertIsPositive(
          "initialDomainCapacity", initialDomainCapacity);
      this.initialDomainCapacity = initialDomainCapacity;
    }

  }

  /**
   * Estimator for a LogDefaultDataDistribution
   * 
   * @param <KeyType>
   *          Type of Key in the distribution
   */
  public static class Estimator<KeyType>
      extends
      AbstractBatchAndIncrementalLearner<KeyType, LogDefaultDataDistribution.PMF<KeyType>>
      implements
      DistributionEstimator<KeyType, LogDefaultDataDistribution.PMF<KeyType>> {

    private static final long serialVersionUID = 8787720132790311008L;

    /**
     * Default constructor
     */
    public Estimator() {
      super();
    }

    @Override
    public LogDefaultDataDistribution.PMF<KeyType> createInitialLearnedObject() {
      return new LogDefaultDataDistribution.PMF<KeyType>();
    }

    @Override
    public void update(
      final LogDefaultDataDistribution.PMF<KeyType> target,
      final KeyType data) {
      target.increment(data, 1.0);
    }

  }

  /**
   * PMF of the LogDefaultDataDistribution
   * 
   * @param <KeyType>
   *          Type of Key in the distribution
   */
  public static class PMF<KeyType> extends
      LogDefaultDataDistribution<KeyType> implements
      DataDistribution.PMF<KeyType> {

    private static final long serialVersionUID = 355507596128913991L;

    /**
     * Default constructor
     */
    public PMF() {
      super();
    }

    /**
     * Copy constructor
     * 
     * @param other
     *          ScalarDataDistribution to copy
     */
    public PMF(final DataDistribution<KeyType> other) {
      super(other);
    }

    /**
     * Creates a new instance of LogDefaultDataDistribution
     * 
     * @param initialCapacity
     *          Initial capacity of the Map
     */
    public PMF(int initialCapacity) {
      super(initialCapacity);
    }

    /**
     * Creates a new instance of ScalarDataDistribution
     * 
     * @param data
     *          Data to create the distribution
     */
    public PMF(final Iterable<? extends KeyType> data) {
      super(data);
    }

    @Override
    public Double evaluate(KeyType input) {
      return this.getFraction(input);
    }

    @Override
    public LogDefaultDataDistribution.PMF<KeyType> getProbabilityFunction() {
      return this;
    }

    @Override
    public double logEvaluate(KeyType input) {
      return this.getLogFraction(input);
    }

  }

  /**
   * A weighted estimator for a LogDefaultDataDistribution
   * 
   * @param <KeyType>
   *          Type of Key in the distribution
   */
  public static class WeightedEstimator<KeyType>
      extends
      AbstractBatchAndIncrementalLearner<WeightedValue<? extends KeyType>, LogDefaultDataDistribution.PMF<KeyType>>
      implements
      DistributionWeightedEstimator<KeyType, LogDefaultDataDistribution.PMF<KeyType>> {

    private static final long serialVersionUID = -9067384837227173014L;

    /**
     * Default constructor
     */
    public WeightedEstimator() {
      super();
    }

    @Override
    public LogDefaultDataDistribution.PMF<KeyType> createInitialLearnedObject() {
      return new LogDefaultDataDistribution.PMF<KeyType>();
    }

    @Override
    public void update(
      final LogDefaultDataDistribution.PMF<KeyType> target,
      final WeightedValue<? extends KeyType> data) {
      target.increment(data.getValue(), data.getWeight());
    }

  }

  private static final long serialVersionUID = 6596579376152342279L;

  /**
   * Default initial capacity, {@value} .
   */
  public static final int DEFAULT_INITIAL_CAPACITY = 10;

  /**
   * Total of the counts in the distribution
   */
  protected double total;

  private int totalCount = 0;

  private LinkedHashMap<KeyType, Multiset.Entry<KeyType>> map;

  /**
   * Default constructor
   */
  public LogDefaultDataDistribution() {
    this(DEFAULT_INITIAL_CAPACITY);
    //    this.total = Double.NEGATIVE_INFINITY;
  }

  /**
   * Creates a new instance of LogDefaultDataDistribution
   * 
   * @param other
   *          DataDistribution to copy
   */
  public LogDefaultDataDistribution(
    final DataDistribution<? extends KeyType> other) {
    this(Maps.<KeyType, Entry<KeyType>>newLinkedHashMap(), 0.0);
    //    this(new LinkedHashMap<KeyType, MutableDouble>(other.size()), Double.NEGATIVE_INFINITY);
    this.incrementAll(other);
  }

  /**
   * Creates a new instance of LogDefaultDataDistribution
   * 
   * @param initialCapacity
   *          Initial capacity of the Map
   */
  public LogDefaultDataDistribution(int initialCapacity) {
    this(Maps.<KeyType, Entry<KeyType>>newLinkedHashMap(), 0.0);
    //    this(new LinkedHashMap<KeyType, MutableDouble>(initialCapacity),
    //        Double.NEGATIVE_INFINITY);
  }

  public LogDefaultDataDistribution(
    final Iterable<? extends KeyType> data) {
    this();
    this.incrementAll(data);
  }

  /**
   * Creates a new instance of
   * 
   * @param map
   *          Backing Map that stores the data
   * @param total
   *          Sum of all values in the Map
   */
  protected LogDefaultDataDistribution(
    final LinkedHashMap<KeyType, Multiset.Entry<KeyType>> map, final double total) {
    this.map = Maps.newLinkedHashMap(map);
    this.total = total;
  }

  @Override
  public void clear() {
    this.map.clear();
    this.total = 0.0;
    //    this.total = Double.NEGATIVE_INFINITY;
    this.keysToCounts.clear();
    this.totalCount = 0;
  }

  @Override
  public LogDefaultDataDistribution<KeyType> clone() {
//    final LogDefaultDataDistribution<KeyType> clone = (LogDefaultDataDistribution<KeyType>) super
//        .clone();
    final LogDefaultDataDistribution<KeyType> clone = new LogDefaultDataDistribution<KeyType>(this.getDomainSize());

    // We have to manually reset "total" because super.super.clone
    // calls "incrementAll", which will, in turn, increment the total
    // So we'd end up with twice the total.
    clone.incrementAll(this.getDomain());
    clone.total = this.total;
    clone.totalCount = this.totalCount;
    return clone;
  }

  public int getCount(final KeyType key) {
    return keysToCounts.get(key);
  }

  @Override
  public DistributionEstimator<KeyType, ? extends DataDistribution<KeyType>> getEstimator() {
    return new LogDefaultDataDistribution.Estimator<KeyType>();
  }

  /**
   * Gets the average value of all keys in the distribution, that is, the total
   * value divided by the number of keys (even zero-value keys)
   * 
   * @return Average value of all keys in the distribution
   */
  public double getMeanValue() {
    final int ds = this.getDomainSize();
    if (ds > 0) {
      return this.getTotal() / ds;
    } else {
      return 0.0;
    }
  }

  @Override
  public DataDistribution.PMF<KeyType> getProbabilityFunction() {
    return new LogDefaultDataDistribution.PMF<KeyType>(this);
  }

  @Override
  public double getTotal() {
    return this.total;
  }

  public int getTotalCount() {
    return totalCount;
  }

  @Override
  public double increment(KeyType key) {
    return this.increment(key, 1d);
  }

  @Override
  public double increment(KeyType key, final double value) {
    final MutableDouble entry = this.map.get(key);
    double newValue;
    double delta;
    if (entry == null) {
      if (value > 0.0) {
        // It's best to avoid this.set() here as it could mess up
        // our total tracker in some subclasses...
        // Also it's more efficient this way (avoid another get)
        this.map.put(key, new MutableDouble(value));
        delta = value;
      } else {
        delta = 0.0;
      }
      newValue = value;
    } else {
      if (entry.value + value >= 0.0) {
        delta = value;
        entry.value += value;
      } else {
        delta = -entry.value;
        entry.value = 0.0;
      }
      newValue = entry.value;
    }

    if (delta != 0.0) {
      keysToCounts.adjustOrPutValue(key, 1, 1);
      totalCount++;
      assert (totalCount == StatisticsUtil.sum(keysToCounts.values()));
    }

    this.total += delta;
    return newValue;
  }

  @Override
  public void set(final KeyType key, final double value) {

    // I decided not to call super.set because it would result in me
    // having to perform an extra call to this.map.get
    final MutableDouble entry = this.map.get(key);
    if (entry == null) {
      // Only need to allocate if it's not null
      if (value > 0.0) {
        this.map.put(key, new MutableDouble(value));
        this.total += value;
        keysToCounts.adjustOrPutValue(key, 1, 1);
        totalCount++;
      }
    } else if (value > 0.0) {
      this.total += value - entry.value;
      entry.value = value;
      keysToCounts.adjustOrPutValue(key, 1, 1);
      totalCount++;
    } else {
      entry.value = 0.0;
    }
    // FIXME TODO debug, remove.
    assert (totalCount <= 50);
    assert (totalCount == StatisticsUtil.sum(keysToCounts.values()));
  }
  @Override
  public int getDomainSize()
  {
      return this.map.size();
  }

  @Override
  public double getEntropy()
  {
      double entropy = 0.0;
      final double total = this.getTotal();
      final double denom = (total != 0.0) ? total : 1.0;
      for (ScalarMap.Entry<KeyType> entry : this.entrySet())
      {
          double p = entry.getValue() / denom;
          if (p != 0.0)
          {
              entropy -= p * MathUtil.log2(p);
          }
      }
      return entropy;
  }

  @Override
  public double getLogFraction(
      KeyType key)
  {
      final double total = this.getTotal();
      return (total != 0.0) ? (Math.log(this.get(key)) - Math.log(total))
          : Double.NEGATIVE_INFINITY;
  }

  @Override
  public double getFraction(
      KeyType key)
  {
      final double total = this.getTotal();
      return (total != 0.0) ? (this.get(key) / this.getTotal()) : 0.0;
  }

  @Override
  public KeyType sample(
      Random random)
  {
      double w = random.nextDouble() * this.getTotal();
      for (ScalarMap.Entry<KeyType> entry : this.entrySet())
      {
          w -= entry.getValue();
          if (w <= 0.0)
          {
              return entry.getKey();
          }
      }
      return null;
  }

  @Override
  public ArrayList<KeyType> sample(
      Random random,
      int numSamples)
  {
      // Compute the cumulative weights
      final int size = this.getDomainSize();
      double[] cumulativeWeights = new double[size];
      double cumulativeSum = 0.0;
      ArrayList<KeyType> domain = new ArrayList<KeyType>(size);
      int index = 0;
      for (ScalarMap.Entry<KeyType> entry : this.entrySet())
      {
          domain.add(entry.getKey());
          final double value = entry.getValue();
          cumulativeSum += value;
          cumulativeWeights[index] = cumulativeSum;
          index++;
      }

      return ProbabilityMassFunctionUtil.sampleMultiple(
          cumulativeWeights, cumulativeSum, domain, random, numSamples);
  }

  @Override
  public InfiniteVector<KeyType> toInfiniteVector()
  {
      final DefaultInfiniteVector<KeyType> result =
          new DefaultInfiniteVector<KeyType>(this.size());

      for (ScalarMap.Entry<KeyType> entry : this.entrySet())
      {
          result.set(entry.getKey(), entry.getValue());
      }

      return result;
  }

  @Override
  public void fromInfiniteVector(
      final InfiniteVector<? extends KeyType> vector)
  {
      this.clear();

      for (ScalarMap.Entry<? extends KeyType> entry : vector.entrySet())
      {
          this.set(entry.getKey(), entry.getValue());
      }
  }

  @Override
  public double getMaxValue()
  {
      double result = super.getMaxValue();
      if( result == Double.NEGATIVE_INFINITY )
      {
          return 0.0;
      }
      else
      {
          return result;
      }
  }

  @Override
  public double getMinValue()
  {
      double result = super.getMinValue();
      if( result == Double.POSITIVE_INFINITY )
      {
          return 0.0;
      }
      else
      {
          return result;
      }
  }

  @Override
  public Set<KeyType> getDomain()
  {
      return this.keySet();
  }
 

}
