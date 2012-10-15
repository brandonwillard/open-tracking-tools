package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseVector;
import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;
import gov.sandia.cognition.util.CloneableSerializable;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

import jj2000.j2k.NotImplementedError;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.graph.paths.InferredPath;
import org.openplans.tools.tracking.impl.graph.paths.PathEdge;
import org.openplans.tools.tracking.impl.statistics.OnOffEdgeTransDirMulti;
import org.openplans.tools.tracking.impl.statistics.StatisticsUtil;
import org.openplans.tools.tracking.impl.statistics.filters.AbstractRoadTrackingFilter;
import org.openplans.tools.tracking.impl.statistics.filters.AdjKalmanFilter;
import org.openplans.tools.tracking.impl.util.OtpGraph;

import com.google.common.annotations.Beta;
import com.google.common.base.Preconditions;

/**
 * This class represents the state of a vehicle, which is made up of the
 * vehicles location, whether it is on an edge, which path it took from its
 * previous location on an edge, and the distributions that determine these.
 * 
 * @author bwillard
 * 
 */
public class VehicleState implements
    ComputableDistribution<Observation>, Comparable<VehicleState> {

  public static class PDF extends VehicleState implements
      ProbabilityFunction<Observation> {

    private static final long serialVersionUID = 879217079360170446L;

    public PDF(VehicleState state) {
      super(state);
    }

    @Override
    public PDF clone() {
      return (PDF) super.clone();
    }

    @Override
    public Double evaluate(Observation input) {
      return Math.exp(logEvaluate(input));
    }

    @Override
    public VehicleState.PDF getProbabilityFunction() {
      return this;
    }

    @Override
    public double logEvaluate(Observation input) {
      double logLikelihood = 0d;

      /*
       * This doesn't belong in the likelihood, naturally,
       * because it has nothing to do with the observation.
       */
//      /*
//       * Edge transitions.
//       */
//      final InferredEdge parentEdge = this.getParentState() != null ? 
//          this.getParentState().getInferredEdge() : null;
//      logLikelihood +=
//          this.edgeTransitionDist.logEvaluate(parentEdge,
//              this.getInferredEdge());

      /*
       * Movement.
       */
      logLikelihood +=
          this.getMovementFilter().logLikelihood(
              input.getProjectedPoint(), this.belief.getMean(),
              PathEdge.getEdge(this.getInferredEdge(), 0d, this.getPath().getIsBackward()));

      return logLikelihood;
    }

    @Override
    public Observation sample(Random random) {
      throw new NotImplementedError();
    }

    @Override
    public ArrayList<Observation>
        sample(Random random, int numSamples) {
      throw new NotImplementedError();
    }

  }

  public static class VehicleStateInitialParameters extends AbstractCloneableSerializable {
    private static final long serialVersionUID = 3613725475525876941L;
    private final Vector obsCov;
    private final Vector onRoadStateCov;
    private final Vector offRoadStateCov;
    private final Vector offTransitionProbs;
    private final Vector onTransitionProbs;
    private final long seed;
    private final int numParticles;
    private final String filterTypeName;
    private final int initialObsFreq;
    private final int obsCovDof;
    private final int onRoadCovDof;
    private final int offRoadCovDof;

    public VehicleStateInitialParameters(
      Vector obsCov, int obsCovDof,
      Vector onRoadStateCov, int onRoadCovDof, 
      Vector offRoadStateCov, int offRoadCovDof,
      Vector offProbs, Vector onProbs, String filterTypeName,
      int numParticles, int initialObsFreq, long seed) {
      this.obsCovDof = obsCovDof;
      this.onRoadCovDof = onRoadCovDof;
      this.offRoadCovDof = offRoadCovDof;
      this.numParticles = numParticles;
      this.obsCov = obsCov;
      this.onRoadStateCov = onRoadStateCov;
      this.offRoadStateCov = offRoadStateCov;
      this.offTransitionProbs = offProbs;
      this.onTransitionProbs = onProbs;
      this.seed = seed;
      this.filterTypeName = filterTypeName;
      this.initialObsFreq = initialObsFreq;
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
      VehicleStateInitialParameters other =
          (VehicleStateInitialParameters) obj;
      if (filterTypeName == null) {
        if (other.filterTypeName != null) {
          return false;
        }
      } else if (!filterTypeName.equals(other.filterTypeName)) {
        return false;
      }
      if (initialObsFreq != other.initialObsFreq) {
        return false;
      }
      if (numParticles != other.numParticles) {
        return false;
      }
      if (obsCov == null) {
        if (other.obsCov != null) {
          return false;
        }
      } else if (!obsCov.equals(other.obsCov)) {
        return false;
      }
      if (obsCovDof != other.obsCovDof) {
        return false;
      }
      if (offRoadCovDof != other.offRoadCovDof) {
        return false;
      }
      if (offRoadStateCov == null) {
        if (other.offRoadStateCov != null) {
          return false;
        }
      } else if (!offRoadStateCov.equals(other.offRoadStateCov)) {
        return false;
      }
      if (offTransitionProbs == null) {
        if (other.offTransitionProbs != null) {
          return false;
        }
      } else if (!offTransitionProbs.equals(other.offTransitionProbs)) {
        return false;
      }
      if (onRoadCovDof != other.onRoadCovDof) {
        return false;
      }
      if (onRoadStateCov == null) {
        if (other.onRoadStateCov != null) {
          return false;
        }
      } else if (!onRoadStateCov.equals(other.onRoadStateCov)) {
        return false;
      }
      if (onTransitionProbs == null) {
        if (other.onTransitionProbs != null) {
          return false;
        }
      } else if (!onTransitionProbs.equals(other.onTransitionProbs)) {
        return false;
      }
      if (seed != other.seed) {
        return false;
      }
      return true;
    }

    public String getFilterTypeName() {
      return filterTypeName;
    }

    public int getNumParticles() {
      return numParticles;
    }

    public Vector getObsCov() {
      return obsCov;
    }

    public Vector getOffRoadStateCov() {
      return offRoadStateCov;
    }

    public Vector getOffTransitionProbs() {
      return offTransitionProbs;
    }

    public Vector getOnRoadStateCov() {
      return onRoadStateCov;
    }

    public Vector getOnTransitionProbs() {
      return onTransitionProbs;
    }

    public long getSeed() {
      return seed;
    }

    @Override
    public int hashCode() {
      final int prime = 31;
      int result = 1;
      result =
          prime
              * result
              + ((filterTypeName == null) ? 0 : filterTypeName
                  .hashCode());
      result = prime * result + initialObsFreq;
      result = prime * result + numParticles;
      result =
          prime * result + ((obsCov == null) ? 0 : obsCov.hashCode());
      result = prime * result + obsCovDof;
      result = prime * result + offRoadCovDof;
      result =
          prime
              * result
              + ((offRoadStateCov == null) ? 0 : offRoadStateCov
                  .hashCode());
      result =
          prime
              * result
              + ((offTransitionProbs == null) ? 0
                  : offTransitionProbs.hashCode());
      result = prime * result + onRoadCovDof;
      result =
          prime
              * result
              + ((onRoadStateCov == null) ? 0 : onRoadStateCov
                  .hashCode());
      result =
          prime
              * result
              + ((onTransitionProbs == null) ? 0 : onTransitionProbs
                  .hashCode());
      result = prime * result + (int) (seed ^ (seed >>> 32));
      return result;
    }

    public int getInitialObsFreq() {
      return this.initialObsFreq;
    }

    public int getObsCovDof() {
      return this.obsCovDof;
    }

    public int getOnRoadCovDof() {
      return onRoadCovDof;
    }

    public int getOffRoadCovDof() {
      return offRoadCovDof;
    }

    @Override
    public VehicleStateInitialParameters clone() {
      VehicleStateInitialParameters clone = (VehicleStateInitialParameters) super.clone();
      // TODO
      return clone;
    }

    @Override
    public String toString() {
      StringBuilder builder = new StringBuilder();
      builder.append("VehicleStateInitialParameters [obsCov=")
          .append(obsCov).append(", onRoadStateCov=")
          .append(onRoadStateCov).append(", offRoadStateCov=")
          .append(offRoadStateCov).append(", offTransitionProbs=")
          .append(offTransitionProbs).append(", onTransitionProbs=")
          .append(onTransitionProbs).append(", seed=").append(seed)
          .append(", numParticles=").append(numParticles)
          .append(", filterTypeName=").append(filterTypeName)
          .append(", initialObsFreq=").append(initialObsFreq)
          .append(", obsCovDof=").append(obsCovDof)
          .append(", onRoadCovDof=").append(onRoadCovDof)
          .append(", offRoadCovDof=").append(offRoadCovDof)
          .append("]");
      return builder.toString();
    }
  }

  private static final long serialVersionUID = 3229140254421801273L;

  /*
   * These members represent the state/parameter samples/sufficient statistics.
   */
  private final AbstractRoadTrackingFilter<?> movementFilter;

  /**
   * This could be the 4D ground-coordinates dist. for free motion, or the 2D
   * road-coordinates, either way the tracking filter will check. Also, this
   * could be the prior or prior predictive distribution.
   */
  protected final MultivariateGaussian belief;

  private final MultivariateGaussian initialBelief;

  /*-
   * Edge transition priors 
   * 1. edge off 
   * 2. edge on 
   * 3. edges transitions to others (one for all)
   * edges
   */
  protected final OnOffEdgeTransDirMulti edgeTransitionDist;
  private final Observation observation;
  private final InferredEdge edge;
  private VehicleState parentState = null;
  private final Double distanceFromPreviousState;

  private final OtpGraph graph;

  // private final int initialHashCode;
  // private final int edgeInitialHashCode;
  // private final int obsInitialHashCode;
  // private final int transInitialHashCode;
  // private final int beliefInitialHashCode;

  private final InferredPath path;

  private int hash = 0;

  public VehicleState(OtpGraph graph, Observation observation,
    AbstractRoadTrackingFilter<?> updatedFilter,
    MultivariateGaussian belief,
    OnOffEdgeTransDirMulti edgeTransitionDist, InferredPath path,
    VehicleState state) {

    Preconditions
        .checkArgument(!(belief.getInputDimensionality() == 2 && path
            .isEmptyPath()));
    Preconditions.checkNotNull(state);
    Preconditions.checkNotNull(graph);
    Preconditions.checkNotNull(observation);
    Preconditions.checkNotNull(updatedFilter);
    Preconditions.checkNotNull(belief);

    this.observation = observation;
    this.movementFilter = updatedFilter;
    this.belief = belief.clone();
    this.graph = graph;
    this.path = path;
    /*
     * This is the constructor used when creating transition states, so this is
     * where we'll need to reset the distance measures
     */
    if (this.belief.getInputDimensionality() == 2) {
      /*
       * IMPORTANT: the filtering that occurs on edges doesn't mean
       * that the filtered location will end up on the same edge.
       */
      final double distPosition = this.belief.getMean().getElement(0);
      final PathEdge pathEdge =
          path.getEdgeForDistance(distPosition, false);

      assert !path.isEmptyPath() && !pathEdge.isEmptyEdge();

      this.edge = pathEdge.getInferredEdge();
      
      if (state != null && !state.path.isEmptyPath()) {
        final Vector startState = path.convertToStateOnPath(
            state.belief.getMean(), state.getInferredEdge());
        this.distanceFromPreviousState =
            distPosition - startState.getElement(0);
      } else {
        this.distanceFromPreviousState = null;
      }

      /*
       * We normalized the position relative to the direction of motion.
       */
      final InferredPath tmpPath = InferredPath.getInferredPath(
          PathEdge.getEdge(pathEdge.getEdge(), 0d, pathEdge.isBackward()));
      this.belief.getMean().setElement(0,
          tmpPath.clampToPath(distPosition - pathEdge.getDistToStartOfEdge()));
      
//      AbstractRoadTrackingFilter.normalizeBelief(
//          this.belief.getMean(), 
//          PathEdge.getEdge(edge, 0d, this.belief.getMean().getElement(0) < 0d));

      assert PathEdge.getEdge(edge, 0d, this.path.getIsBackward())
        .isOnEdge(this.belief.getMean().getElement(0));

    } else {

      this.edge = InferredEdge.getEmptyEdge();
      this.distanceFromPreviousState = null;
    }

    /*
     * This has to come after the adjustment 
     */
    this.initialBelief = this.belief.clone();

    this.edgeTransitionDist = edgeTransitionDist;

    this.parentState = state;
    /*
     * Reset the parent's parent state so that we don't keep these objects
     * forever.
     */
    // state.parentState = null;

    final double timeDiff;
    if (observation.getPreviousObservation() != null) {
      timeDiff =
          (observation.getTimestamp().getTime() - observation
              .getPreviousObservation().getTimestamp().getTime()) / 1000d;
    } else {
      timeDiff = 30d;
    }
    this.movementFilter.setCurrentTimeDiff(timeDiff);

    // DEBUG
    // this.initialHashCode = this.hashCode();
    // this.edgeInitialHashCode = this.edge.hashCode();
    // this.transInitialHashCode = this.edgeTransitionDist.hashCode();
    // this.beliefInitialHashCode =
    // Arrays.hashCode(((DenseVector)this.initialBelief.convertToVector()).getArray());
    // this.obsInitialHashCode = this.observation.hashCode();
  }

  public VehicleState(OtpGraph graph, Observation initialObservation,
    InferredEdge inferredEdge,
    AbstractRoadTrackingFilter<?> movementFilter,
    OnOffEdgeTransDirMulti edgeTransDist, Random rng) {

    Preconditions.checkNotNull(initialObservation);
    Preconditions.checkNotNull(inferredEdge);

    this.movementFilter = movementFilter;

    final double timeDiff;
    if (initialObservation.getPreviousObservation() != null) {
      timeDiff =
          (initialObservation.getTimestamp().getTime() - initialObservation
              .getPreviousObservation().getTimestamp().getTime()) / 1000d;
    } else {
      timeDiff = movementFilter.getCurrentTimeDiff();
    }
    Preconditions.checkArgument(timeDiff > 0d);
    this.movementFilter.setCurrentTimeDiff(timeDiff);


    if (inferredEdge.isEmptyEdge()) {
      this.belief =
          movementFilter.getGroundFilter()
              .createInitialLearnedObject();
      final Vector xyPoint = initialObservation.getProjectedPoint();

      /*
       * Sample the velocity from the initial prior.
       * TODO: does this initial dist make sense?  the covar
       * is the transition variance...
       */
//      final Matrix beliefCov = belief.getCovariance().clone();
//      final Vector stateSmpl = MultivariateGaussian.sample(belief.getMean(), 
//          StatisticsUtil.getCholR(beliefCov).transpose(), rng);
      final Vector stateSmpl = movementFilter.sampleStateTransDist(belief.getMean(), rng);
      
      belief.setMean(stateSmpl);
      belief.getMean().setElement(0, xyPoint.getElement(0));
      belief.getMean().setElement(2, xyPoint.getElement(1));
      
      this.path = InferredPath.getEmptyPath();

    } else {
      /*
       * Find our starting position on this edge
       */
      this.belief =
          movementFilter.getRoadFilter().createInitialLearnedObject();

      final double lengthLocation =
          inferredEdge.getLengthIndexedLine().project(
              initialObservation.getObsPoint());

      /*
       * Sample the velocity
       */
//      final Matrix beliefCov = belief.getCovariance().clone();
//      final Vector stateSmpl = MultivariateGaussian.sample(belief.getMean(), 
//          StatisticsUtil.getCholR(beliefCov), rng);
      final Vector stateSmpl = this.movementFilter.sampleStateTransDist(belief.getMean(),
          rng);
      belief.setMean(stateSmpl);
      belief.getMean().setElement(0, lengthLocation);

      final PathEdge pathEdge = PathEdge.getEdge(inferredEdge, 0d, lengthLocation < 0d);
      this.path = InferredPath.getInferredPath(pathEdge);

      assert pathEdge.isOnEdge(lengthLocation);
      
    }

    this.edge = inferredEdge;
    this.initialBelief = belief.clone();
    this.observation = initialObservation;
    this.graph = graph;
    this.edgeTransitionDist = edgeTransDist;
    this.distanceFromPreviousState = 0d;

    // DEBUG
    // this.initialHashCode = this.hashCode();
    // this.edgeInitialHashCode = this.edge.hashCode();
    // this.transInitialHashCode = this.edgeTransitionDist.hashCode();
    // this.beliefInitialHashCode =
    // Arrays.hashCode(((DenseVector)this.initialBelief.convertToVector()).getArray());
    // this.obsInitialHashCode = this.observation.hashCode();
  }

  public VehicleState(VehicleState other) {
    this.graph = other.graph;
    this.movementFilter = other.movementFilter.clone();
    this.belief = other.belief.clone();
    this.edgeTransitionDist = other.edgeTransitionDist.clone();
    // TODO clone this?
    this.edge = other.edge;
    this.observation = other.observation;
    this.distanceFromPreviousState = other.distanceFromPreviousState;
    this.parentState = other.parentState;
    this.initialBelief = other.initialBelief.clone();
    this.path = other.path;

    // DEBUG
    // this.initialHashCode = this.hashCode();
    // this.edgeInitialHashCode = this.edge.hashCode();
    // this.transInitialHashCode = this.edgeTransitionDist.hashCode();
    // this.beliefInitialHashCode =
    // Arrays.hashCode(((DenseVector)this.initialBelief.convertToVector()).getArray());
    // this.obsInitialHashCode = this.observation.hashCode();
  }

  @Override
  public VehicleState clone() {
    return new VehicleState(this);
  }

  @Override
  public int compareTo(VehicleState arg0) {
    return oneStateCompareTo(this, arg0);
  }

  @Override
  public boolean equals(Object obj) {
    /*
     * We do this to avoid evaluating every parent down the chain.
     */
    if (!oneStateEquals(this, obj))
      return false;
  
    VehicleState other = (VehicleState) obj;
    if (parentState == null) {
      if (other.parentState != null) {
        return false;
      }
    } else if ( !oneStateEquals(parentState, other.parentState)) {
      return false;
    }
    
    return true;
  }
  
  protected static boolean oneStateEquals(Object thisObj, Object obj) {
    if (thisObj == obj) {
      return true;
    }
    if (obj == null) {
      return false;
    }
    if (thisObj.getClass() != obj.getClass()) {
      return false;
    }
    VehicleState thisState = (VehicleState) obj;
    VehicleState other = (VehicleState) obj;
    if (thisState.belief == null) {
      if (other.belief != null) {
        return false;
      }
    } else if (!thisState.belief.equals(other.belief)) {
      return false;
    }
    if (thisState.edge == null) {
      if (other.edge != null) {
        return false;
      }
    } else if (!thisState.edge.equals(other.edge)) {
      return false;
    }
    if (thisState.edgeTransitionDist == null) {
      if (other.edgeTransitionDist != null) {
        return false;
      }
    } else if (!thisState.edgeTransitionDist.equals(other.edgeTransitionDist)) {
      return false;
    }
    if (thisState.movementFilter == null) {
      if (other.movementFilter != null) {
        return false;
      }
    } else if (!thisState.movementFilter.equals(other.movementFilter)) {
      return false;
    }
    if (thisState.observation == null) {
      if (other.observation != null) {
        return false;
      }
    } else if (!thisState.observation.equals(other.observation)) {
      return false;
    }
    if (thisState.path == null) {
      if (other.path != null) {
        return false;
      }
    } else if (!thisState.path.equals(other.path)) {
      return false;
    }
    return true;
  }

  public MultivariateGaussian getBelief() {
    return belief;
  }

  public Double getDistanceFromPreviousState() {
    return distanceFromPreviousState;
  }

  public InferredEdge getEdge() {
    return edge;
  }

  public OnOffEdgeTransDirMulti getEdgeTransitionDist() {
    return edgeTransitionDist;
  }

  public OtpGraph getGraph() {
    return graph;
  }

  /**
   * Gets the belief in ground coordinates (even if it's really tracking road
   * coordinates).
   * 
   * @return
   */
  public MultivariateGaussian getGroundOnlyBelief() {
    if (belief.getInputDimensionality() == 2) {
      final MultivariateGaussian beliefProj =
          new MultivariateGaussian();
      AbstractRoadTrackingFilter.convertToGroundBelief(beliefProj,
          PathEdge.getEdge(this.edge, 0d, this.path.getIsBackward()),
          true);
      return beliefProj;
    } else {
      return belief;
    }
  }

  public InferredEdge getInferredEdge() {
    return this.edge;
  }

  public MultivariateGaussian getInitialBelief() {
    return initialBelief;
  }

  public AdjKalmanFilter getKalmanFilter() {
    return this.belief.getInputDimensionality() == 4 ? this
        .getMovementFilter().getGroundFilter() : this
        .getMovementFilter().getRoadFilter();
  }

  /**
   * Returns ground-coordinate mean location
   * 
   * @return
   */
  public Vector getMeanLocation() {
    Vector v;
    if (belief.getInputDimensionality() == 2) {
      Preconditions.checkArgument(!this.edge.isEmptyEdge());
      final MultivariateGaussian projBelief = belief.clone();
      AbstractRoadTrackingFilter.convertToGroundBelief(projBelief,
          PathEdge.getEdge(this.edge, 0d, this.path.getIsBackward()),
          true);
      v = projBelief.getMean();
    } else {
      v = belief.getMean();
    }
    return VectorFactory.getDefault().createVector2D(v.getElement(0),
        v.getElement(2));
  }

  public AbstractRoadTrackingFilter<?> getMovementFilter() {
    return movementFilter;
  }

  public Vector getNonVelocityVector() {
    return getNonVelocityVector(this.belief.getMean());
  }

  public Observation getObservation() {
    return observation;
  }

  public VehicleState getParentState() {
    return parentState;
  }

  public InferredPath getPath() {
    return path;
  }

  @Override
  public VehicleState.PDF getProbabilityFunction() {
    return new VehicleState.PDF(this);
  }

  @Override
  public int hashCode() {
    /*
     * We do this to avoid evaluating every parent down the chain.
     */
    if (hash != 0) {
      return hash;
    } else {
      final int prime = 31;
      int result = 1;
      result = prime *result + oneStateHashCode(this);
      if (this.parentState != null)
        result = prime *result + oneStateHashCode(this.parentState);
      hash = result;
      return result;
    }
  }
  
  protected static int oneStateHashCode(VehicleState state) {
    final int prime = 31;
    int result = 1;
    result =
        prime * result + ((state.belief == null) ? 0 : state.belief.hashCode());
    result = prime * result + ((state.edge == null) ? 0 : state.edge.hashCode());
    result =
        prime
            * result
            + ((state.edgeTransitionDist == null) ? 0 : state.edgeTransitionDist
                .hashCode());
    result =
        prime
            * result
            + ((state.movementFilter == null) ? 0 : state.movementFilter
                .hashCode());
    result =
        prime * result
            + ((state.observation == null) ? 0 : state.observation.hashCode());
    result = prime * result + ((state.path == null) ? 0 : state.path.hashCode());
    return result;
  }

  @Override
  public Observation sample(Random random) {
    throw new NotImplementedError();
  }

  @Override
  public ArrayList<Observation> sample(Random random, int numSamples) {
    throw new NotImplementedError();
  }

  public void setParentState(VehicleState parentState) {
    this.parentState = parentState;
  }

  @Override
  public String toString() {
    StringBuilder builder = new StringBuilder();
    builder.append("VehicleState [belief=").append(belief)
        .append(", observation=").append(observation)
        .append(", edge=").append(edge).append(", path=")
        .append(path).append("]");
    return builder.toString();
  }

  public static Vector getNonVelocityVector(Vector vector) {
    final Vector res;
    if (vector.getDimensionality() == 4)
      res = AbstractRoadTrackingFilter.getOg().times(vector);
    else
      res = AbstractRoadTrackingFilter.getOr().times(vector);
    return res;
  }

  public static long getSerialversionuid() {
    return serialVersionUID;
  }

  private static int
      oneStateCompareTo(VehicleState t, VehicleState o) {
    if (t == o)
      return 0;

    if (t == null) {
      if (o != null)
        return -1;
      else
        return 0;
    } else if (o == null) {
      return 1;
    }

    final DenseVector thisV =
        (DenseVector) t.initialBelief.convertToVector();
    final DenseVector otherV =
        (DenseVector) o.initialBelief.convertToVector();
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(thisV.getArray(), otherV.getArray());
    comparator.append(t.getObservation(), o.getObservation());
    comparator.append(t.edge, o.edge);
    comparator.append(t.edgeTransitionDist, o.edgeTransitionDist);

    return comparator.toComparison();
  }

}
