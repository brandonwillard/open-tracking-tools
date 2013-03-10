package org.opentrackingtools.distributions;

import gov.sandia.cognition.collection.ScalarMap;
import gov.sandia.cognition.factory.Factory;
import gov.sandia.cognition.learning.algorithm.AbstractBatchAndIncrementalLearner;
import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.MathUtil;
import gov.sandia.cognition.math.MutableDouble;
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

import org.opentrackingtools.util.model.MutableDoubleCount;

import com.google.common.base.Preconditions;
import com.google.common.primitives.Doubles;

/**
 * Copy of DefaultDataDistribution that an object count and provides an
 * configurable scale (log or not).
 * 
 * @author bwillard
 * 
 * @param <KeyType>
 */
public class DefaultCountedDataDistribution<KeyType> extends
    AbstractDataDistribution<KeyType> {

  /**
   * A factory for {@code DefaultCountedDataDistribution} objects using some
   * given initial capacity for them.
   * 
   * @param <DataType>
   *          The type of data for the factory.
   */
  public static class DefaultFactory<DataType> extends
      AbstractCloneableSerializable implements
      Factory<DefaultCountedDataDistribution<DataType>> {

    /**
       * 
       */
    private static final long serialVersionUID = 681699655965182747L;
    /** The initial domain capacity. */
    protected int initialDomainCapacity;
    private boolean isLogScale;

    /**
     * Creates a new {@code DefaultFactory} with a default initial domain
     * capacity.
     */
    public DefaultFactory(boolean isLogScale) {
      this(DefaultCountedDataDistribution.DEFAULT_INITIAL_CAPACITY);
      this.isLogScale = isLogScale;
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
    public DefaultCountedDataDistribution<DataType> create() {
      // Create the histogram.
      return new DefaultCountedDataDistribution<DataType>(
          this.getInitialDomainCapacity(), this.isLogScale);
    }

    /**
     * Gets the initial domain capacity.
     * 
     * @return The initial domain capacity. Must be positive.
     */
    public int getInitialDomainCapacity() {
      return this.initialDomainCapacity;
    }

    public boolean isLogScale() {
      return this.isLogScale;
    }

    /**
     * Sets the initial domain capacity.
     * 
     * @param initialDomainCapacity
     *          The initial domain capacity. Must be positive.
     */
    public void setInitialDomainCapacity(
      final int initialDomainCapacity) {
      ArgumentChecker.assertIsPositive("initialDomainCapacity",
          initialDomainCapacity);
      this.initialDomainCapacity = initialDomainCapacity;
    }

    public void setLogScale(boolean isLogScale) {
      this.isLogScale = isLogScale;
    }

  }

  /**
   * Estimator for a DefaultCountedDataDistribution
   * 
   * @param <KeyType>
   *          Type of Key in the distribution
   */
  public static class Estimator<KeyType>
      extends
      AbstractBatchAndIncrementalLearner<KeyType, DefaultCountedDataDistribution.PMF<KeyType>>
      implements
      DistributionEstimator<KeyType, DefaultCountedDataDistribution.PMF<KeyType>> {

    /**
       * 
       */
    private static final long serialVersionUID = 8787720132790311008L;

    protected boolean isLogScale;

    /**
     * Default constructor
     */
    public Estimator(boolean isLogScale) {
      super();
      this.isLogScale = isLogScale;
    }

    @Override
    public Estimator<KeyType> clone() {
      final Estimator<KeyType> clone =
          (Estimator<KeyType>) super.clone();
      clone.isLogScale = this.isLogScale;
      return clone;
    }

    @Override
    public DefaultCountedDataDistribution.PMF<KeyType>
        createInitialLearnedObject() {
      return new DefaultCountedDataDistribution.PMF<KeyType>(
          this.isLogScale);
    }

    @Override
    public void update(
      final DefaultCountedDataDistribution.PMF<KeyType> target,
      final KeyType data) {
      target.increment(data, this.isLogScale ? 0d : 1d);
    }

  }

  /**
   * PMF of the DefaultCountedDataDistribution
   * 
   * @param <KeyType>
   *          Type of Key in the distribution
   */
  public static class PMF<KeyType> extends
      DefaultCountedDataDistribution<KeyType> implements
      DataDistribution.PMF<KeyType> {

    /**
       * 
       */
    private static final long serialVersionUID = 355507596128913991L;

    /**
     * Default constructor
     */
    public PMF(boolean isLogScale) {
      super(isLogScale);
    }

    /**
     * Copy constructor
     * 
     * @param other
     *          ScalarDataDistribution to copy
     */
    public PMF(final DataDistribution<KeyType> other,
      boolean isLogScale) {
      super(other, isLogScale);
    }

    /**
     * Creates a new instance of DefaultCountedDataDistribution
     * 
     * @param initialCapacity
     *          Initial capacity of the Map
     */
    public PMF(int initialCapacity, boolean isLogScale) {
      super(initialCapacity, isLogScale);
    }

    /**
     * Creates a new instance of ScalarDataDistribution
     * 
     * @param data
     *          Data to create the distribution
     */
    public PMF(final Iterable<? extends KeyType> data,
      boolean isLogScale) {
      super(data, isLogScale);
    }

    @Override
    public Double evaluate(KeyType input) {
      return this.getFraction(input);
    }

    @Override
    public DefaultCountedDataDistribution.PMF<KeyType>
        getProbabilityFunction() {
      return this;
    }

    @Override
    public double logEvaluate(KeyType input) {
      return this.getLogFraction(input);
    }

  }

  /**
   * A weighted estimator for a DefaultCountedDataDistribution
   * 
   * @param <KeyType>
   *          Type of Key in the distribution
   */
  public static class WeightedEstimator<KeyType>
      extends
      AbstractBatchAndIncrementalLearner<WeightedValue<? extends KeyType>, DefaultCountedDataDistribution.PMF<KeyType>>
      implements
      DistributionWeightedEstimator<KeyType, DefaultCountedDataDistribution.PMF<KeyType>> {

    /**
       * 
       */
    private static final long serialVersionUID =
        -9067384837227173014L;

    private final boolean isLogScaled;

    /**
     * Default constructor
     */
    public WeightedEstimator(boolean isLogScaled) {
      super();
      this.isLogScaled = isLogScaled;
    }

    @Override
    public DefaultCountedDataDistribution.PMF<KeyType>
        createInitialLearnedObject() {
      return new DefaultCountedDataDistribution.PMF<KeyType>(
          this.isLogScaled);
    }

    @Override
    public void update(
      final DefaultCountedDataDistribution.PMF<KeyType> target,
      final WeightedValue<? extends KeyType> data) {
      //      Preconditions.checkArgument(target.isLogScale && isLogScaled);
      Preconditions.checkArgument(!this.isLogScaled
          || data.getWeight() <= 0d);
      target.increment(data.getValue(), data.getWeight());
    }

  }

  /**
   * Default initial capacity, {@value} .
   */
  public static final int DEFAULT_INITIAL_CAPACITY = 10;

  /**
   * 
   */
  private static final long serialVersionUID = 6596579376152342279L;

  protected boolean isLogScale = false;

  /**
   * Total of the counts in the distribution
   */
  protected double total;

  /**
   * Default constructor
   */
  public DefaultCountedDataDistribution(boolean isLogScale) {
    this(DefaultCountedDataDistribution.DEFAULT_INITIAL_CAPACITY,
        isLogScale);
  }

  /**
   * Creates a new instance of DefaultCountedDataDistribution
   * 
   * @param other
   *          DataDistribution to copy
   */
  public DefaultCountedDataDistribution(
    final DataDistribution<? extends KeyType> other,
    boolean isLogScale) {
    this(new LinkedHashMap<KeyType, MutableDouble>(other.size()),
        isLogScale ? Double.NEGATIVE_INFINITY : 0d, isLogScale);
    this.incrementAll(other);
  }

  /**
   * Creates a new instance of DefaultCountedDataDistribution
   * 
   * @param initialCapacity
   *          Initial capacity of the Map
   */
  public DefaultCountedDataDistribution(int initialCapacity,
    boolean isLogScale) {
    this(new LinkedHashMap<KeyType, MutableDouble>(initialCapacity),
        isLogScale ? Double.NEGATIVE_INFINITY : 0d, isLogScale);
    this.isLogScale = isLogScale;
  }

  /**
   * Creates a new instance of ScalarDataDistribution
   * 
   * @param data
   *          Data to create the distribution
   */
  public DefaultCountedDataDistribution(
    final Iterable<? extends KeyType> data, boolean isLogScale) {
    this(isLogScale);
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
  protected DefaultCountedDataDistribution(
    final Map<KeyType, MutableDouble> map, final double total,
    boolean isLogScale) {
    super(map);
    this.isLogScale = isLogScale;
    this.total = total;
  }

  @Override
  public void clear() {
    super.clear();
    this.total = this.isLogScale ? Double.NEGATIVE_INFINITY : 0d;
  }

  @Override
  public DefaultCountedDataDistribution<KeyType> clone() {
    final DefaultCountedDataDistribution<KeyType> clone =
        new DefaultCountedDataDistribution<KeyType>(this.size(),
            this.isLogScale);
    for (final java.util.Map.Entry<KeyType, MutableDouble> entry : this.map
        .entrySet()) {
      final MutableDoubleCount count =
          (MutableDoubleCount) entry.getValue();
      clone.set(entry.getKey(), count.getValue(), count.getCount());
    }

    assert this.getTotalCount() == clone.getTotalCount();

    clone.total = this.total;
    return clone;
  }

  public void copyAll(DataDistribution<KeyType> posteriorDist) {
    for (final java.util.Map.Entry<KeyType, ? extends Number> entry : posteriorDist
        .asMap().entrySet()) {
      final MutableDoubleCount count =
          (MutableDoubleCount) entry.getValue();
      this.set(entry.getKey(), count.getValue(), count.getCount());
    }
  }

  public int getCount(KeyType key) {
    final MutableDoubleCount count =
        ((MutableDoubleCount) this.map.get(key));
    return count != null ? count.getCount() : 0;
  }

  /**
   * This value does not take duplicates into account.
   */
  @Override
  public int getDomainSize() {
    return super.getDomainSize();
  }

  @Override
  public double getEntropy() {
    final double identity =
        this.isLogScale ? Double.NEGATIVE_INFINITY : 0d;
    double entropy = identity;
    final double total = this.getTotal();
    final double denom =
        (Doubles.compare(total, identity) != 0) ? total
            : (this.isLogScale ? 0d : 1d);
    for (final ScalarMap.Entry<KeyType> entry : this.entrySet()) {
      if (this.isLogScale) {
        final double p = entry.getValue() - denom;
        if (Doubles.compare(p, identity) != 0) {
          entropy =
              LogMath
                  .subtract(entropy, p + p / MathUtil.log2(Math.E));
        }

      } else {
        final double p = entry.getValue() / denom;
        if (Doubles.compare(p, identity) != 0) {
          entropy -= p * MathUtil.log2(p);
        }
      }
    }
    return entropy;
  }

  @Override
  public
      DistributionEstimator<KeyType, ? extends DataDistribution<KeyType>>
      getEstimator() {
    return new DefaultCountedDataDistribution.Estimator<KeyType>(
        this.isLogScale);
  }

  @Override
  public double getFraction(KeyType key) {
    if (this.isLogScale) {
      return Math.exp(this.getLogFraction(key));
    } else {
      return super.getFraction(key);
    }
  }

  @Override
  public double getLogFraction(KeyType key) {
    if (this.isLogScale) {
      return this.get(key) - this.getTotal();
    } else {
      return super.getLogFraction(key);
    }
  }

  /**
   * Gets the average value of all keys in the distribution, that is, the total
   * value divided by the number of keys (even zero-value keys)
   * 
   * @return Average value of all keys in the distribution
   */
  public double getMeanValue() {
    final int ds = this.getTotalCount();
    if (ds > 0) {
      return (this.isLogScale ? Math.exp(this.getTotal()) : this
          .getTotal()) / ds;
    } else {
      return 0.0;
    }
  }

  @Override
  public DataDistribution.PMF<KeyType> getProbabilityFunction() {
    return new DefaultCountedDataDistribution.PMF<KeyType>(this,
        this.isLogScale);
  }

  @Override
  public double getTotal() {
    return this.total;
  }

  /**
   * @return the total number of elements, including duplicates
   */
  public int getTotalCount() {
    int total = 0;
    for (final MutableDouble value : this.map.values()) {
      final MutableDoubleCount count = (MutableDoubleCount) value;
      total += count.getCount();
    }
    return total;
  }

  @Override
  public double increment(KeyType key, final double value) {
    return this.increment(key, value, 1);
  }

  public double increment(KeyType key, final double value, int count) {
    // TODO FIXME terrible hack!
    final MutableDoubleCount entry =
        (MutableDoubleCount) this.map.get(key);
    double newValue;
    double delta;
    final double identity =
        this.isLogScale ? Double.NEGATIVE_INFINITY : 0d;
    if (entry == null) {
      if (value > identity) {
        this.map.put(key, new MutableDoubleCount(value, count));
        delta = value;
      } else {
        delta = identity;
      }
      newValue = value;
    } else {
      final double sum =
          this.isLogScale ? LogMath.add(entry.value, value)
              : entry.value + value;
      if (sum >= identity) {
        delta = value;
        entry.set(sum, entry.count + count);
      } else {
        delta = -entry.value;
        entry.set(identity);
      }
      newValue = entry.value;
    }

    if (this.isLogScale) {
      this.total = LogMath.add(this.total, value);
    } else {
      this.total += delta;
    }

    return newValue;
  }

  public boolean isLogScale() {
    return this.isLogScale;
  }

  @Override
  public KeyType sample(Random random) {
    if (this.isLogScale) {
      double w = random.nextDouble();
      for (final ScalarMap.Entry<KeyType> entry : this.entrySet()) {
        w -= this.getFraction(entry.getKey());
        if (w <= 0d) {
          return entry.getKey();
        }
      }
      return null;
    } else {
      return super.sample(random);
    }
  }

  @Override
  public ArrayList<KeyType> sample(Random random, int numSamples) {
    if (this.isLogScale) {
      // Compute the cumulative weights
      final int size = this.getDomainSize();
      final double[] cumulativeWeights = new double[size];
      double cumulativeSum = 0d;
      final ArrayList<KeyType> domain = new ArrayList<KeyType>(size);
      int index = 0;
      for (final ScalarMap.Entry<KeyType> entry : this.entrySet()) {
        domain.add(entry.getKey());
        final double value = entry.getValue();
        cumulativeSum += Math.exp(value);
        cumulativeWeights[index] = cumulativeSum;
        index++;
      }

      return ProbabilityMassFunctionUtil.sampleMultiple(
          cumulativeWeights, cumulativeSum, domain, random,
          numSamples);

    } else {
      return super.sample(random, numSamples);
    }
  }

  @Override
  public void set(final KeyType key, final double value) {
    // TODO FIXME terrible hack!
    final MutableDoubleCount entry =
        (MutableDoubleCount) this.map.get(key);
    this.set(key, value, entry == null ? 1 : entry.getCount());
  }

  /**
   * In this case the given value is total across counts. There is a basic, but
   * loose, assumption that each count * key has value = totalValue/count.
   * 
   * @param key
   * @param totalValue
   * @param count
   */
  public void set(final KeyType key, final double totalValue,
    final int count) {

    // TODO FIXME terrible hack!
    final MutableDoubleCount entry =
        (MutableDoubleCount) this.map.get(key);
    final double identity =
        this.isLogScale ? Double.NEGATIVE_INFINITY : 0d;
    if (entry == null) {
      // Only need to allocate if it's not null
      if (totalValue > identity) {
        this.map.put(key, new MutableDoubleCount(totalValue, count));

        if (this.isLogScale) {
          this.total = LogMath.add(this.total, totalValue);
        } else {
          this.total += totalValue;
        }
      }
    } else if (totalValue > identity) {
      if (this.isLogScale) {
        if (entry.value > totalValue) {
          final double totalsSum =
              LogMath.add(this.total, totalValue);
          Preconditions.checkState(totalsSum >= entry.value);
          this.total = LogMath.subtract(totalsSum, entry.value);
        } else {
          this.total =
              LogMath.add(this.total,
                  LogMath.subtract(totalValue, entry.value));
        }
      } else {
        this.total += totalValue - entry.value;
      }
      entry.set(totalValue, count);
    } else {
      /*
       * 'set' does not do a relative adjustment of the value
       * so we should check the value here.
       */
      Preconditions.checkArgument(Doubles.compare(totalValue,
          identity) == 0);
      entry.set(identity, count);
    }
  }

}
